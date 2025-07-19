#include <iostream>
#include <vector>
#include <thread>
#include <queue>
#include <map>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <random>
#include <cmath>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <fstream>   // For writing results to file
using namespace std;

enum CollisionMode { DEAD, BOUNCE };
CollisionMode collisionMode = BOUNCE;
atomic<bool> stopSignal(false);
int num_collisions = 0;
int num_dead = 0; // Counter for dead drones

class Logger {
    static mutex logMutex;
public:
    static void log(const string& message) {
        lock_guard<mutex> lock(logMutex);
        cout << message << endl;
    }
};
mutex Logger::logMutex;

struct Coordinate {
    int x, y, z;
    bool operator!=(const Coordinate& other) const {
        return x != other.x || y != other.y || z != other.z;
    }
    bool operator==(const Coordinate& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct DroneState {
    Coordinate current;
    Coordinate destination;
    atomic<long> timestamp;
    atomic<bool> alive;
    atomic<bool> atDestination; // Track if drone reached destination
    
    DroneState() : timestamp(0), alive(true), atDestination(false) {}
    DroneState(const DroneState& other) {
        current = other.current;
        destination = other.destination;
        timestamp.store(other.timestamp.load());
        alive.store(other.alive.load());
        atDestination.store(other.atDestination.load());
    }
    
    DroneState& operator=(const DroneState& other) {
        if (this != &other) {
            current = other.current;
            destination = other.destination;
            timestamp.store(other.timestamp.load());
            alive.store(other.alive.load());
            atDestination.store(other.atDestination.load());
        }
        return *this;
    }
};

struct CRDTMessage {
    int fromDroneId;
    map<int, DroneState> crdtData;
};

class MessageQueue {
    queue<CRDTMessage> messages;
    mutex mtx;
    condition_variable cv;

public:
    void push(CRDTMessage msg) {
        unique_lock<mutex> lock(mtx);
        messages.push(move(msg));
        cv.notify_one();
    }

    bool try_pop(CRDTMessage& msg) {
        unique_lock<mutex> lock(mtx);
        if (messages.empty()) return false;
        msg = move(messages.front());
        messages.pop();
        return true;
    }

    bool empty() {
        unique_lock<mutex> lock(mtx);
        return messages.empty();
    }
};

class Drone {
    public:
        const int id;
        Coordinate start, destination;
        map<int, DroneState> localCRDT;
        MessageQueue inbox;
    
        static vector<Drone*> allDrones;
        static mutex dronesMutex;
        atomic<bool> alive;
        bool inCircleFormation;
        Coordinate circleCenter;
        double circleRadius;
        double circleTheta;
        int totalCircleDrones;
    
        Drone(int id, Coordinate start, Coordinate destination)
            : id(id), start(start), destination(destination),
              alive(true), inCircleFormation(false) {
            localCRDT[id].current = start;
            localCRDT[id].destination = destination;
            localCRDT[id].atDestination = false;
            {
                lock_guard<mutex> lock(dronesMutex);
                allDrones.push_back(this);
            }
        }
    
        ~Drone() = default;
    
        Drone(const Drone&) = delete;
        Drone& operator=(const Drone&) = delete;
    
        void prepareForCircleFormation(Coordinate center, double radius, double theta, int total, int angleStep) {
            unique_lock<mutex> lock(dronesMutex);
            if (!alive.load()) return;
    
            inCircleFormation = true;
            circleCenter = center;
            circleRadius = radius;
            circleTheta = theta;
            totalCircleDrones = total;
    
            double radians = (theta-angleStep/2) * M_PI / 180.0;
            Coordinate dest = {
                static_cast<int>(round(center.x + radius * cos(radians))),
                static_cast<int>(round(center.y + radius * sin(radians))),
                center.z
            };
    
            lock.unlock();
    
            localCRDT[id].destination = dest;
            localCRDT[id].atDestination = false;
            start = localCRDT[id].current;
    
            stringstream circleMsg;
            circleMsg << "[DRONE " << id << "] 🌀 CIRCLE formation - θ=" << fixed << setprecision(2) << theta 
                      << "° → Destination: (" << dest.x << ", "
                      << dest.y << ", " << dest.z << ")";
            Logger::log(circleMsg.str());
        }
    
        Coordinate calculateNextMove() {
            Coordinate curr = localCRDT[id].current;
            Coordinate dest = localCRDT[id].destination;
            if (curr == dest) return curr;
    
            Coordinate next = curr;
            if (curr.x != dest.x) {
                if(curr.x < dest.x)
                next.x+=1;
                else if(curr.x> dest.x)
                next.x+=-1;
            }
            if (curr.y != dest.y) {
                if(curr.y < dest.y)
                next.y+=1;
                else if(curr.y> dest.y)
                next.y+=-1;
            }
            if (curr.z != dest.z) 
            {
                if(curr.z < dest.z)
                next.z+=1;
                else if(curr.z> dest.z)
                next.z+=-1;
            }
            return next;
        }
    
        void mergeCRDT(const map<int, DroneState>& otherCRDT) {
            for (const auto& [droneId, state] : otherCRDT) {
                auto& myState = localCRDT[droneId];
                if (myState.timestamp.load() < state.timestamp.load()) {
                    myState = state;
                }
            }
        }
    
        void broadcast() {
            if (!alive.load()) return;
            
            CRDTMessage msg;
            msg.fromDroneId = id;
            {
                lock_guard<mutex> lock(dronesMutex);
                msg.crdtData = localCRDT;
            }
    
            vector<Drone*> dronesToNotify;
            {
                lock_guard<mutex> lock(dronesMutex);
                for (Drone* other : allDrones) {
                    if (other && other->id != id && other->alive.load()) {
                        dronesToNotify.push_back(other);
                    }
                }
            }
    
            for (Drone* other : dronesToNotify) {
                if (other) {
                    other->inbox.push(msg);
                }
            }
        }
    
        void processMessages() {
            CRDTMessage msg;
            while (inbox.try_pop(msg)) {
                mergeCRDT(msg.crdtData);
            }
        }
    
        bool checkCollision(const Coordinate& next) {
            lock_guard<mutex> lock(dronesMutex);
            for (Drone* other : allDrones) {
                if (!other || other->id == id || !other->alive.load()) continue;
                
                Coordinate otherPos = other->localCRDT[other->id].current;
                if (otherPos == next) {
                    num_collisions++;
                    return true;
                }
            }
            return false;
        }
    
        void run() {
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<> delay(50, 150);
        
            while (alive.load() && !stopSignal.load()) {
                // Phase 1: Process messages
                processMessages();
        
                // Phase 2: Check if at destination
                Coordinate current, dest;
                bool atDest;
                {
                    lock_guard<mutex> lock(dronesMutex);
                    current = localCRDT[id].current;
                    dest = localCRDT[id].destination;
                    atDest = (current == dest);
                    if (atDest) {
                        localCRDT[id].atDestination = true;
                    }
                }
        
                // If already at destination, broadcast final state and stop
                if (atDest) {
                    broadcast();  // Ensure other drones know this drone has reached its destination
                    return;       // Exit the run method
                }
        
                // Phase 3: Calculate next position
                Coordinate next = calculateNextMove();
        
                // Phase 4: Collision detection
                bool collision = false;
                int collidedWith = -1;
                {
                    lock_guard<mutex> lock(dronesMutex);
                    for (Drone* other : allDrones) {
                        if (!other || other->id == id || !other->alive.load()) continue;
                        if (other->localCRDT[other->id].current == next) {
                            collision = true;
                            collidedWith = other->id;
                            num_collisions++;
                            break;
                        }
                    }
                }
        
                // Phase 5: Handle collision
                if (collision && collidedWith != -1) {
                    handleCollision(collidedWith, next);
                    continue;
                }
        
                // Phase 6: Normal movement
                moveTo(next);
        
                // Phase 7: Broadcast state
                broadcast();
                this_thread::sleep_for(chrono::milliseconds(delay(gen)));
            }
        }
    
    private:
        void generateBounceCoordinates(Coordinate& myBounce, Coordinate& theirBounce) {
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<> dist(-2, 2);
    
            Coordinate myCurrent = localCRDT[id].current;
            Coordinate theirCurrent;
    
            {
                lock_guard<mutex> lock(dronesMutex);
                for (Drone* d : allDrones) {
                    if (d && d->id != id) {
                        theirCurrent = d->localCRDT[d->id].current;
                        break;
                    }
                }
            }
    
            int attempts = 0;
            do {
                myBounce = {
                    max(0, myCurrent.x + dist(gen)),
                    max(0, myCurrent.y + dist(gen)),
                    max(0, myCurrent.z + dist(gen))
                };
                theirBounce = {
                    max(0, theirCurrent.x + dist(gen)),
                    max(0, theirCurrent.y + dist(gen)),
                    max(0, theirCurrent.z + dist(gen))
                };
    
                bool myOccupied = false, theirOccupied = false;
                {
                    lock_guard<mutex> lock(dronesMutex);
                    for (const auto& d : allDrones) {
                        if (!d || d->id == id || !d->alive.load()) continue;
                        if (d->localCRDT[d->id].current == myBounce) myOccupied = true;
                        if (d->localCRDT[d->id].current == theirBounce) theirOccupied = true;
                    }
                }
    
                if (!myOccupied && !theirOccupied && myBounce != theirBounce) break;
                attempts++;
    
            } while (attempts < 10);
        }
    
        void handleCollision(int collidedWith, const Coordinate& collisionPoint) {
            stringstream collisionMsg;
            collisionMsg << "💥 [COLLISION] Drone " << id << " and Drone " << collidedWith 
                        << " at (" << collisionPoint.x << ", " << collisionPoint.y << ", " << collisionPoint.z << ")";
            Logger::log(collisionMsg.str());
    
            if (collisionMode == DEAD) {
                // Mark ourselves dead
                {
                    lock_guard<mutex> lock(dronesMutex);
                    alive = false;
                    localCRDT[id].alive = false;
                    num_dead++;
                }
                
                // Try to mark other drone dead
                Drone* otherDrone = nullptr;
                {
                    lock_guard<mutex> lock(dronesMutex);
                    auto it = find_if(allDrones.begin(), allDrones.end(),
                        [collidedWith](Drone* d) { return d && d->id == collidedWith; });
                    if (it != allDrones.end()) {
                        otherDrone = *it;
                    }
                }
    
                if (otherDrone && otherDrone->alive.load()) {
                    bool expected = true;
                    if (otherDrone->alive.compare_exchange_strong(expected, false)) {
                        {
                            lock_guard<mutex> lock(dronesMutex);
                            otherDrone->localCRDT[collidedWith].alive = false;
                            num_dead++;
                        }
                        
                        stringstream deathMsg;
                        deathMsg << "☠️ [COLLISION RESOLUTION] Drones " << id << " and " 
                                << collidedWith << " have been terminated";
                        Logger::log(deathMsg.str());
                    }
                }
                
                broadcast();
            } 
            else if (collisionMode == BOUNCE) {
                Coordinate myNewPos, theirNewPos;
                generateBounceCoordinates(myNewPos, theirNewPos);
    
                Logger::log("[BOUNCE INITIATOR] Drone " + to_string(id) + " initiating bounce");
    
                // Move self first (atomic operation)
                {
                    lock_guard<mutex> lock(dronesMutex);
                    localCRDT[id].current = myNewPos;
                    localCRDT[id].timestamp++;
                }
                
                stringstream moveMsg;
                moveMsg << "[DRONE " << id << "] ↗️ Moving to (" 
                       << myNewPos.x << ", " << myNewPos.y << ", " << myNewPos.z << ")";
                Logger::log(moveMsg.str());
    
                // Prepare bounce message for other drone
                CRDTMessage bounceMsg;
                bounceMsg.fromDroneId = id;
                {
                    lock_guard<mutex> lock(dronesMutex);
                    bounceMsg.crdtData = localCRDT;
                    bounceMsg.crdtData[collidedWith].current = theirNewPos;
                    bounceMsg.crdtData[collidedWith].timestamp++;
                }
    
                // Find and notify other drone
                Drone* otherDrone = nullptr;
                {
                    lock_guard<mutex> lock(dronesMutex);
                    auto it = find_if(allDrones.begin(), allDrones.end(),
                        [collidedWith](Drone* d) { return d && d->id == collidedWith; });
                    if (it != allDrones.end()) {
                        otherDrone = *it;
                    }
                }
    
                if (otherDrone) {
                    bool otherAtDest;
                    {
                        lock_guard<mutex> lock(dronesMutex);
                        otherAtDest = otherDrone->localCRDT[collidedWith].atDestination;
                    }
    
                    if (!otherAtDest) {
                        otherDrone->inbox.push(bounceMsg);
                        stringstream bounceMsg2;
                        bounceMsg2 << "[DRONE " << collidedWith << "] 🔀 Bouncing to (" 
                                   << theirNewPos.x << ", " << theirNewPos.y << ", " << theirNewPos.z << ")";
                        Logger::log(bounceMsg2.str());
                    }
                }
            }
        }
    
        void moveTo(const Coordinate& next) {
            if (!alive.load()) return;
            
            localCRDT[id].current = next;
            localCRDT[id].timestamp++;
            broadcast();
    
            stringstream moveMsg;
            moveMsg << "[DRONE " << id << "] ↗️ Moving to (" << next.x << ", " << next.y << ", " << next.z << ")";
            Logger::log(moveMsg.str());
        }
    };

vector<Drone*> Drone::allDrones;
mutex Drone::dronesMutex;

void cleanupDeadDrones() {
    lock_guard<mutex> lock(Drone::dronesMutex);
    auto it = remove_if(Drone::allDrones.begin(), Drone::allDrones.end(),
        [](Drone* d) {
            if (d && !d->alive.load()) {
                delete d;
                return true;
            }
            return false;
        });
    Drone::allDrones.erase(it, Drone::allDrones.end());
}

vector<Drone*> getAliveDrones() {
    lock_guard<mutex> lock(Drone::dronesMutex);
    vector<Drone*> aliveDrones;
    for (Drone* d : Drone::allDrones) {
        if (d && d->alive.load()) {
            aliveDrones.push_back(d);
        }
    }
    return aliveDrones;
}

bool allDronesCompleted() {
    lock_guard<mutex> lock(Drone::dronesMutex);
    for (Drone* d : Drone::allDrones) {
        if (d) {
            // Only check alive drones
            if (d->alive.load()) {
                bool atDestination = (d->localCRDT[d->id].current == d->localCRDT[d->id].destination);
                if (!atDestination) {
                    return false;
                }
            }
        }
    }
    return true;
}
void simulateDrones(int numDrones, CollisionMode mode, double radius) {
    stopSignal = false;
    num_collisions = 0;
    num_dead = 0;
    Drone::allDrones.clear();

    collisionMode = mode;

    vector<Coordinate> initialPositions(numDrones);
    int x_coord = -numDrones/4, y_coord = 0;
    int id = numDrones/4 + 1;
    for (int i = 1; i <= numDrones/2; i++, id++) {
        initialPositions[id].x = x_coord;
        initialPositions[id].y = y_coord;
        initialPositions[id].z = 0;
        x_coord -= numDrones/4;
        y_coord -= numDrones/4;
    }
    id = numDrones/4;
    x_coord = numDrones/4;
    y_coord = 0;
    for (int i = 1; i <= numDrones/2; i++) {
        initialPositions[id].x = x_coord;
        initialPositions[id].y = y_coord;
        initialPositions[id].z = 0;
        id--;
        if (id == -1) id = numDrones - 1;
        x_coord += numDrones/4;
        y_coord -= numDrones/4;
    }

    vector<Drone*> drones;
    for (int i = 0; i < numDrones; ++i) {
        drones.push_back(new Drone(i, initialPositions[i], initialPositions[i]));
    }

    double circleCenterX = 0;
    double circleCenterY = ((numDrones/4)+1) * radius;
    double circleCenterZ = 0;
    Coordinate circleCenter = { (int)circleCenterX, (int)circleCenterY, (int)circleCenterZ };

    double angleStep = 360.0 / numDrones;
    vector<thread> prepareThreads;
    for (int i = 0; i < numDrones; i++) {
        double theta = i * angleStep;
        prepareThreads.emplace_back(&Drone::prepareForCircleFormation, drones[i],
                                    circleCenter, radius*numDrones/4, theta, numDrones, angleStep);
    }
    for (auto& t : prepareThreads) t.join();

    vector<thread> droneThreads;
    for (Drone* d : drones) {
        droneThreads.emplace_back(&Drone::run, d);
    }

    thread monitorThread([]() {
        while (!stopSignal.load()) {
            this_thread::sleep_for(chrono::milliseconds(100));
            if (allDronesCompleted()) {
                stopSignal = true;
                break;
            }
        }
    });

    for (auto& t : droneThreads) t.join();
    monitorThread.join();
    cleanupDeadDrones();
    ofstream viz("drone_positions.csv");
    viz << "id,x_init,y_init,x_final,y_final\n";
    for (Drone* d : getAliveDrones()) {
        const auto& state = d->localCRDT[d->id];
        viz << d->id << ","
            << d->start.x << "," << d->start.y << ","
            << state.current.x << "," << state.current.y << "\n";
    }
    viz.close();
}
pair<int, int> runAveragedExperiment(int numDrones, CollisionMode mode, double radius, int trials) {
    int totalCollisions = 0;
    int totalDeaths = 0;

    for (int i = 0; i < trials; ++i) {
        simulateDrones(numDrones, mode, radius);
        totalCollisions += num_collisions;
        totalDeaths += num_dead;
    }

    return {
        totalCollisions / trials,
        totalDeaths / trials
    };
}


int main() {
    vector<int> droneCounts = {12};//4,8,12,16,20,24,28,32,36,40
    int trials = 1;//30
    double radius = 4;

    ofstream out("results.csv");
    out << "Drones,Collisions,Deaths\n";

    for (int count : droneCounts) {
        auto [avgBounceCollisions, _] = runAveragedExperiment(count, BOUNCE, radius, trials);
        auto [_, avgDeadDrones] = runAveragedExperiment(count, DEAD, radius, trials);
        out << count << "," << avgBounceCollisions << "," << avgDeadDrones << "\n";

        cout << "✔ Tested " << count << " drones: Avg Collisions = "
             << avgBounceCollisions << ", Avg Dead Drones = " << avgDeadDrones << endl;
    }

    out.close();
    return 0;
}