#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

// ==========================================
// 1. DATA STRUCTURES
// ==========================================
struct Particle {
    Vector3 position;
    Vector3 previous_position;
    Vector3 acceleration;
    float mass;
    bool isWheel;
};

struct Spring {
    Particle* p1;
    Particle* p2;
    float rest_length;
    float stiffness;
    float damping; 
    bool isSuspension; 
    float max_deformation;
};

struct VisualVertex {
    Vector3 position;
    int boundParticleIndex; 
    Vector3 offset;         
};

struct Obstacle {
    Vector3 minBounds;
    Vector3 maxBounds;
    Color color;
};

int pFront = 0, pBack = 0, pTop = 0; 

// ==========================================
// 2. PHYSICS ENGINE
// ==========================================
Vector3 Subtract(Vector3 v1, Vector3 v2) { return {v1.x - v2.x, v1.y - v2.y, v1.z - v2.z}; }
Vector3 Add(Vector3 v1, Vector3 v2) { return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z}; }
Vector3 Scale(Vector3 v, float s) { return {v.x * s, v.y * s, v.z * s}; }
float Dot(Vector3 v1, Vector3 v2) { return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z; }
float Dist(Vector3 v1, Vector3 v2) { Vector3 d = Subtract(v1, v2); return std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z); }
Vector3 Cross(Vector3 v1, Vector3 v2) { return { v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x }; }
Vector3 Norm(Vector3 v) { float l = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z); if(l==0) return {0,0,0}; return {v.x/l, v.y/l, v.z/l}; }

void UpdateParticle(Particle& p, float deltaTime) {
    Vector3 velocity = Subtract(p.position, p.previous_position);
    velocity = Scale(velocity, 0.99f); 
    p.previous_position = p.position;
    Vector3 delta = Add(velocity, Scale(p.acceleration, deltaTime * deltaTime));
    p.position = Add(p.position, delta);
    p.acceleration = {0.0f, 0.0f, 0.0f};
}

void ApplySpringForce(Spring& spring) {
    float curr_dist = Dist(spring.p1->position, spring.p2->position);
    if (curr_dist < 0.05f) curr_dist = 0.05f; // Prevents singularities

    Vector3 dir = Norm(Subtract(spring.p2->position, spring.p1->position));
    float displacement = curr_dist - spring.rest_length;

    // --- CRUMPLE PHYSICS ---
    if (!spring.isSuspension) {
        float yield = 0.2f; // Force required to permanently bend metal
        if (std::abs(displacement) > yield) {
            float deformation = displacement * 0.05f; 
            // Cap deformation so the car crushes but doesn't explode
            if (std::abs(spring.rest_length + deformation - spring.max_deformation) < spring.max_deformation * 0.6f) {
                spring.rest_length += deformation; 
            }
        }
    }

    Vector3 v1 = Subtract(spring.p1->position, spring.p1->previous_position);
    Vector3 v2 = Subtract(spring.p2->position, spring.p2->previous_position);
    float dampForce = Dot(Subtract(v2, v1), dir) * spring.damping;

    float force = (spring.stiffness * displacement) + dampForce;
    Vector3 p1Force = Scale(dir, force / spring.p1->mass);
    Vector3 p2Force = Scale(dir, force / spring.p2->mass);

    spring.p1->acceleration = Add(spring.p1->acceleration, p1Force);
    spring.p2->acceleration = Subtract(spring.p2->acceleration, p2Force);
}

void HandleCollision(Particle& p, const Obstacle& obs) {
    // 3D AABB Collision Check
    if (p.position.x > obs.minBounds.x && p.position.x < obs.maxBounds.x &&
        p.position.y > obs.minBounds.y && p.position.y < obs.maxBounds.y &&
        p.position.z > obs.minBounds.z && p.position.z < obs.maxBounds.z) {
        
        float dx1 = std::abs(p.position.x - obs.minBounds.x);
        float dx2 = std::abs(obs.maxBounds.x - p.position.x);
        float dy1 = std::abs(p.position.y - obs.minBounds.y);
        float dy2 = std::abs(obs.maxBounds.y - p.position.y);
        float dz1 = std::abs(p.position.z - obs.minBounds.z);
        float dz2 = std::abs(obs.maxBounds.z - p.position.z);

        float m = std::min({dx1, dx2, dy1, dy2, dz1, dz2});

        if(m==dx1) p.position.x = obs.minBounds.x; else if(m==dx2) p.position.x = obs.maxBounds.x;
        else if(m==dy1) p.position.y = obs.minBounds.y; else if(m==dy2) p.position.y = obs.maxBounds.y;
        else if(m==dz1) p.position.z = obs.minBounds.z; else if(m==dz2) p.position.z = obs.maxBounds.z;
        
        // This completely kills momentum on impact, causing the rear of the car to crush into the front
        p.previous_position = p.position; 
    }
}

void ApplyTireFriction(Particle& wheel, Vector3 heading, float grip) {
    Vector3 vel = Subtract(wheel.position, wheel.previous_position);
    
    // Ignore Y-axis (suspension bounce)
    Vector3 planarVel = {vel.x, 0.0f, vel.z}; 
    Vector3 planarHeading = {heading.x, 0.0f, heading.z};
    planarHeading = Norm(planarHeading);

    float forwardSpeed = Dot(planarVel, planarHeading);
    Vector3 forwardVel = Scale(planarHeading, forwardSpeed);
    Vector3 lateralVel = Subtract(planarVel, forwardVel);
    
    // Sideways friction (prevents ice-skating)
    lateralVel = Scale(lateralVel, 1.0f - grip);
    
    Vector3 newPlanar = Add(forwardVel, lateralVel);
    Vector3 finalVel = {newPlanar.x, vel.y, newPlanar.z}; 

    wheel.previous_position = Subtract(wheel.position, finalVel);
}

// ==========================================
// 3. LOADERS
// ==========================================
void LoadPhysicsCage(const char* fileName, std::vector<Particle>& particles, std::vector<Spring>& springs, Vector3 offset) {
    std::ifstream file(fileName); std::string line;
    while(std::getline(file, line)) {
        std::istringstream iss(line); std::string p; iss >> p;
        if(p=="v") { 
            Vector3 v; iss>>v.x>>v.y>>v.z; 
            particles.push_back({Add(v, offset), Add(v, offset), {0,0,0}, 5.0f, false}); 
        }
    }
    for(size_t i=0; i<particles.size(); i++) for(size_t j=i+1; j<particles.size(); j++) {
        float len = Dist(particles[i].position, particles[j].position);
        springs.push_back({&particles[i], &particles[j], len, 1500.0f, 60.0f, false, len});
    }
}

void LoadWheels(const char* fileName, std::vector<Particle>& wheels, std::vector<Particle>& cage, std::vector<Spring>& springs, Vector3 offset) {
    std::ifstream file(fileName); std::string line;
    while(std::getline(file, line)) {
        std::istringstream iss(line); std::string p; iss >> p;
        if(p=="v") { 
            Vector3 v; iss>>v.x>>v.y>>v.z; 
            wheels.push_back({Add(v, offset), Add(v, offset), {0,0,0}, 15.0f, true}); 
        }
    }
    for(auto& w : wheels) for(auto& c : cage) {
        float len = Dist(w.position, c.position);
        springs.push_back({&w, &c, len * 0.95f, 2500.0f, 120.0f, true, len});
    }
}

void LoadSkin(const char* fileName, std::vector<VisualVertex>& skin, std::vector<int>& indices, const std::vector<Particle>& cage, Vector3 offset) {
    std::ifstream file(fileName); std::string line;
    while(std::getline(file, line)) {
        std::istringstream iss(line); std::string p; iss >> p;
        if(p=="v") { 
            Vector3 v; iss>>v.x>>v.y>>v.z; v = Add(v, offset);
            float minD = 9999; int idx = 0;
            for(size_t i=0; i<cage.size(); i++) { float d=Dist(v, cage[i].position); if(d<minD){minD=d; idx=i;} }
            skin.push_back({v, idx, {0,0,0}}); 
        } else if(p=="f") {
            int v1, v2, v3; std::string s; iss >> s; v1=std::stoi(s.substr(0, s.find('/')))-1;
            iss >> s; v2=std::stoi(s.substr(0, s.find('/')))-1; iss >> s; v3=std::stoi(s.substr(0, s.find('/')))-1;
            indices.push_back(v1); indices.push_back(v2); indices.push_back(v3);
        }
    }
    
    // --- FIXED ORIENTATION LOGIC ---
    // Assuming +Z is the front of your car (based on your camera and driving directions)
    float minZ=999, maxZ=-999, maxY=-999;
    for(size_t i=0; i<cage.size(); i++) {
        if(cage[i].position.z > maxZ) { maxZ=cage[i].position.z; pFront=i; } // +Z is Front
        if(cage[i].position.z < minZ) { minZ=cage[i].position.z; pBack=i; }  // -Z is Back
        if(cage[i].position.y > maxY) { maxY=cage[i].position.y; pTop=i; }
    }
    Vector3 fwd = Norm(Subtract(cage[pFront].position, cage[pBack].position));
    Vector3 up = Norm(Cross(fwd, Norm(Subtract(cage[pTop].position, cage[pBack].position))));
    Vector3 right = Norm(Cross(up, fwd));
    for(auto& v : skin) {
        Vector3 local = Subtract(v.position, cage[v.boundParticleIndex].position);
        v.offset = { Dot(local, right), Dot(local, up), Dot(local, fwd) };
    }
}

// ==========================================
// 4. MAIN
// ==========================================
int main() {
    InitWindow(1280, 720, "Vortex Engine V7.0 - The Crash Test");
    SetTargetFPS(60);

    Camera3D camera = { {15.0f, 15.0f, -15.0f}, {0.0f, 2.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 45.0f, CAMERA_PERSPECTIVE };

    std::vector<Particle> cage, wheels;
    std::vector<Spring> springs;
    Vector3 spawn = {0, 2.5f, 0}; 
    LoadPhysicsCage("cage.obj", cage, springs, spawn);
    LoadWheels("wheels.obj", wheels, cage, springs, spawn);
    
    std::vector<VisualVertex> skin; std::vector<int> indices;
    LoadSkin("Car.obj", skin, indices, cage, spawn);

    Model wheelModels[4];
    wheelModels[0] = LoadModel("wheel_fl.obj");
    wheelModels[1] = LoadModel("wheel_fr.obj");
    wheelModels[2] = LoadModel("wheel_rl.obj");
    wheelModels[3] = LoadModel("wheel_rr.obj");

    // Sort wheels based on Z
    std::vector<int> wIdx(4); for(int i=0; i<4; i++) wIdx[i]=i;
    std::sort(wIdx.begin(), wIdx.end(), [&](int a, int b){ return wheels[a].position.z > wheels[b].position.z; });
    int fl=wIdx[0], fr=wIdx[1], rl=wIdx[2], rr=wIdx[3];

    // --- FIXED OBSTACLE BOUNDS ---
    // Solid block from Z=15 to Z=25. It is 10 meters thick. The math can't tunnel through this.
    Obstacle pillar = { {-4.0f, 0.0f, 15.0f}, {4.0f, 10.0f, 25.0f}, GRAY }; 
    
    float steer=0, wheelRot=0;
    float currentGas = 0.0f; // Momentum variable

    while(!WindowShouldClose()) {
        float dt = GetFrameTime();
        if(dt > 0.03f) dt = 0.03f;

        if(!cage.empty()) {
            Vector3 carPos = cage[0].position;
            camera.target = carPos;
            camera.position.x = Lerp(camera.position.x, carPos.x + 12.0f, 0.1f);
            camera.position.y = Lerp(camera.position.y, carPos.y + 8.0f, 0.1f);
            // Camera tracks from behind (-Z)
            camera.position.z = Lerp(camera.position.z, carPos.z - 15.0f, 0.1f); 
        }

        // --- MASSIVE ENGINE POWER ---
        float targetGas = 0.0f;
        if(IsKeyDown(KEY_W)) targetGas = 120000.0f; // High Torque
        if(IsKeyDown(KEY_S)) targetGas = -80000.0f; // Heavy Reverse/Brake
        
        currentGas += (targetGas - currentGas) * 5.0f * dt; // Smooth spool up

        if(IsKeyDown(KEY_A)) steer += 3.0f * dt; else if(IsKeyDown(KEY_D)) steer -= 3.0f * dt; else steer *= 0.8f;
        if(steer > 0.6f) steer=0.6f; if(steer < -0.6f) steer=-0.6f;

        bool handbrake = IsKeyDown(KEY_SPACE);

        Vector3 cFwd = Norm(Subtract(cage[pFront].position, cage[pBack].position));
        Vector3 cUp = Norm(Cross(cFwd, Norm(Subtract(cage[pTop].position, cage[pBack].position))));
        Vector3 cRight = Norm(Cross(cUp, cFwd));
        cUp = Cross(cFwd, cRight);
        
        Vector3 planarFwd = Norm({cFwd.x, 0, cFwd.z});
        Vector3 planarRight = Norm({cRight.x, 0, cRight.z});

        Vector3 steerDir = Add(Scale(planarFwd, cos(steer)), Scale(planarRight, -sin(steer)));
        wheelRot += (Dot(Subtract(wheels[fl].position, wheels[fl].previous_position), planarFwd) / 0.4f); 

        // --- PHYSICS LOOP ---
        int steps = 10; // Increased substeps for tighter collision math
        float subDt = dt/steps;
        for(int s=0; s<steps; s++) {
            
            for(auto& p : cage) p.acceleration.y -= 30.0f; 
            for(auto& w : wheels) w.acceleration.y -= 30.0f;

            for(auto& sp : springs) ApplySpringForce(sp);

            for(auto& p : cage) { 
                UpdateParticle(p, subDt); 
                HandleCollision(p, pillar); 
                if(p.position.y < 0.2f) p.position.y = 0.2f; 
            }
            
            for(int i=0; i<4; i++) {
                // Apply Drive Force
                if(wheels[i].position.y <= 0.6f) { 
                    Vector3 dir = (i==fl || i==fr) ? steerDir : planarFwd;
                    
                    if(!handbrake && (i==rl || i==rr)) { // RWD Power
                        wheels[i].acceleration = Add(wheels[i].acceleration, Scale(dir, currentGas / wheels[i].mass));
                    }
                }

                UpdateParticle(wheels[i], subDt);
                HandleCollision(wheels[i], pillar); 
                
                // Friction
                if(wheels[i].position.y <= 0.55f) {
                    if(wheels[i].position.y < 0.4f) wheels[i].position.y = 0.4f; 
                    
                    Vector3 dir = (i==fl || i==fr) ? steerDir : planarFwd;
                    float grip = 0.9f; 

                    if(handbrake && (i==rl || i==rr)) grip = 0.1f; // Drift
                    
                    ApplyTireFriction(wheels[i], dir, grip);
                }
            }
        }

        for(auto& v : skin) {
            Vector3 pos = cage[v.boundParticleIndex].position;
            v.position = Add(pos, Add(Scale(cRight, v.offset.x), Add(Scale(cUp, v.offset.y), Scale(cFwd, v.offset.z))));
        }

        BeginDrawing();
        ClearBackground(SKYBLUE);
        BeginMode3D(camera);
            DrawGrid(40, 1.0f);
            
            // Draw matching pillar block
            DrawCubeV({0, 5, 20.0f}, {8, 10, 10}, GRAY); 

            // Debug Direction Line
            DrawLine3D(cage[pFront].position, Add(cage[pFront].position, Scale(planarFwd, 3.0f)), GREEN);

            rlBegin(RL_TRIANGLES);
            for(size_t i=0; i<indices.size(); i+=3) {
                Vector3 v1=skin[indices[i]].position, v2=skin[indices[i+1]].position, v3=skin[indices[i+2]].position;
                Vector3 n = Norm(Cross(Subtract(v2, v1), Subtract(v3, v1)));
                float l = fabs(Dot(n, {0,1,0})) * 0.5f + 0.5f;
                rlColor4ub(255*l, 0, 0, 255);
                rlVertex3f(v1.x, v1.y, v1.z); rlVertex3f(v2.x, v2.y, v2.z); rlVertex3f(v3.x, v3.y, v3.z);
                rlVertex3f(v1.x, v1.y, v1.z); rlVertex3f(v3.x, v3.y, v3.z); rlVertex3f(v2.x, v2.y, v2.z);
            }
            rlEnd();
            
            rlBegin(RL_LINES);
            rlColor4ub(0,0,0,255);
            for(size_t i=0; i<indices.size(); i+=3) {
                 Vector3 v1=skin[indices[i]].position, v2=skin[indices[i+1]].position, v3=skin[indices[i+2]].position;
                 rlVertex3f(v1.x, v1.y, v1.z); rlVertex3f(v2.x, v2.y, v2.z);
                 rlVertex3f(v2.x, v2.y, v2.z); rlVertex3f(v3.x, v3.y, v3.z);
                 rlVertex3f(v3.x, v3.y, v3.z); rlVertex3f(v1.x, v1.y, v1.z);
            }
            rlEnd();

            auto DrawOneWheel = [&](Model& m, int physIdx, float angle) {
                Matrix matScale = MatrixScale(1.0f, 1.0f, 1.0f);
                Matrix matTrans = MatrixTranslate(wheels[physIdx].position.x, wheels[physIdx].position.y, wheels[physIdx].position.z);
                Matrix matRot = MatrixMultiply(MatrixRotateX(wheelRot), MatrixRotateY(angle)); 
                float carAngle = atan2(planarFwd.x, planarFwd.z); 
                Matrix matBody = MatrixRotateY(carAngle);
                m.transform = MatrixMultiply(matScale, MatrixMultiply(matRot, MatrixMultiply(matBody, matTrans)));
                DrawModel(m, {0,0,0}, 1.0f, WHITE);
            };

            DrawOneWheel(wheelModels[0], fl, -steer); 
            DrawOneWheel(wheelModels[1], fr, -steer); 
            DrawOneWheel(wheelModels[2], rl, 0);      
            DrawOneWheel(wheelModels[3], rr, 0);      

        EndMode3D();
        DrawFPS(10, 10);
        DrawText("CRASH TEST ACTIVE. HOLD W TO FLOOR IT.", 10, 30, 20, DARKGRAY);
        EndDrawing();
    }
    
    for(int i=0; i<4; i++) UnloadModel(wheelModels[i]);
    CloseWindow();
    return 0;
}