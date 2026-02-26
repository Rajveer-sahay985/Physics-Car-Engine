#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

Vector3 Subtract(Vector3 v1, Vector3 v2) { return {v1.x - v2.x, v1.y - v2.y, v1.z - v2.z}; }
Vector3 Add(Vector3 v1, Vector3 v2)      { return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z}; }
Vector3 Scale(Vector3 v, float s)        { return {v.x * s, v.y * s, v.z * s}; }
float   Dot(Vector3 v1, Vector3 v2)      { return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z; }

Vector3 Normalize(Vector3 v) {
    float length = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (length == 0.0f) return {0,0,0};
    return {v.x/length, v.y/length, v.z/length};
}
Vector3 Cross(Vector3 v1, Vector3 v2) {
    return { v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x };
}
float Distance(Vector3 v1, Vector3 v2) {
    Vector3 d = Subtract(v1, v2);
    return std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
}

struct Particle {
    Vector3 position, previous_position, acceleration;
    float mass;
    bool isWheel;
};
struct Spring {
    Particle* p1; Particle* p2;
    float rest_length, original_length, stiffness, damping;
    bool isSuspension;
};
struct VisualVertex {
    Vector3 position;
    int boundParticleIndex;
    Vector3 offset;
};
struct Obstacle {
    Vector3 minBounds, maxBounds;
    Color color;
    Model model;
    bool hasModel;
    Vector3 position;
};

int pFront = 0, pBack = 0, pTop = 0;

void UpdateParticle(Particle& p, float dt) {
    Vector3 vel = Scale(Subtract(p.position, p.previous_position), 0.99f);
    p.previous_position = p.position;
    p.position.x += vel.x + p.acceleration.x * dt * dt;
    p.position.y += vel.y + p.acceleration.y * dt * dt;
    p.position.z += vel.z + p.acceleration.z * dt * dt;
    p.acceleration = {0,0,0};
}

void ApplyTireFriction(Particle& wheel, Vector3 heading, float grip) {
    Vector3 vel      = Subtract(wheel.position, wheel.previous_position);
    Vector3 fwdVel   = Scale(heading, Dot(vel, heading));
    Vector3 latVel   = Scale(Subtract(vel, fwdVel), 1.0f - grip);
    wheel.previous_position = Subtract(wheel.position, Add(fwdVel, latVel));
}

void ApplySpringForce(Spring& spring, float dt) {
    float dist = Distance(spring.p1->position, spring.p2->position);
    if (dist == 0.0f) return;
    Vector3 dir = Normalize(Subtract(spring.p2->position, spring.p1->position));
    float displacement = dist - spring.rest_length;

    if (!spring.isSuspension) {
        if (std::abs(displacement) > 0.08f) {
            Vector3 v1 = Subtract(spring.p1->position, spring.p1->previous_position);
            Vector3 v2 = Subtract(spring.p2->position, spring.p2->previous_position);
            float impact = std::abs(Dot(Subtract(v2, v1), dir));
            float crush  = displacement * 0.18f * (1.0f + impact * 40.0f);
            if (std::abs(spring.rest_length + crush - spring.original_length) < spring.original_length * 0.65f) {
                spring.rest_length += crush;
                spring.damping = std::min(spring.damping + 0.1f, 25.0f);
            }
        }
    }

    Vector3 v1 = Subtract(spring.p1->position, spring.p1->previous_position);
    Vector3 v2 = Subtract(spring.p2->position, spring.p2->previous_position);
    float damp  = Dot(Subtract(v2, v1), dir) * spring.damping;
    float force = spring.stiffness * displacement + damp;
    spring.p1->acceleration = Add(spring.p1->acceleration, Scale(dir,  force / spring.p1->mass));
    spring.p2->acceleration = Subtract(spring.p2->acceleration, Scale(dir, force / spring.p2->mass));
}

void HandleObstacleCollision(Particle& p, const Obstacle& obs) {
    if (p.position.x <= obs.minBounds.x || p.position.x >= obs.maxBounds.x) return;
    if (p.position.y <= obs.minBounds.y || p.position.y >= obs.maxBounds.y) return;
    if (p.position.z <= obs.minBounds.z || p.position.z >= obs.maxBounds.z) return;

    float dX0 = std::abs(p.position.x - obs.minBounds.x);
    float dX1 = std::abs(obs.maxBounds.x - p.position.x);
    float dY0 = std::abs(p.position.y - obs.minBounds.y);
    float dY1 = std::abs(obs.maxBounds.y - p.position.y);
    float dZ0 = std::abs(p.position.z - obs.minBounds.z);
    float dZ1 = std::abs(obs.maxBounds.z - p.position.z);
    float mn   = std::min({dX0, dX1, dY0, dY1, dZ0, dZ1});

    if      (mn == dX0) { p.position.x = obs.minBounds.x; p.previous_position.x = p.position.x; }
    else if (mn == dX1) { p.position.x = obs.maxBounds.x; p.previous_position.x = p.position.x; }
    else if (mn == dY0) { p.position.y = obs.minBounds.y; p.previous_position.y = p.position.y; }
    else if (mn == dY1) { p.position.y = obs.maxBounds.y; p.previous_position.y = p.position.y; }
    else if (mn == dZ0) { p.position.z = obs.minBounds.z; p.previous_position.z = p.position.z; }
    else                { p.position.z = obs.maxBounds.z; p.previous_position.z = p.position.z; }
}

void CreateSpring(std::vector<Spring>& springs, Particle& p1, Particle& p2,
                  float k, float d, bool susp) {
    float dist = Distance(p1.position, p2.position);
    springs.push_back({&p1, &p2, dist, dist, k, d, susp});
}

void LoadPhysicsCage(const char* file, std::vector<Particle>& parts,
                     std::vector<Spring>& springs, float k, Vector3 offset) {
    std::ifstream f(file); if (!f.is_open()) return;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream iss(line); std::string pre; iss >> pre;
        if (pre == "v") {
            Vector3 p; iss >> p.x >> p.y >> p.z;
            parts.push_back({Add(p, offset), Add(p, offset), {0,0,0}, 1.0f, false});
        }
    }
    for (size_t i = 0; i < parts.size(); i++)
        for (size_t j = i+1; j < parts.size(); j++)
            CreateSpring(springs, parts[i], parts[j], k, 4.0f, false);
}

void LoadWheelsAndSuspension(const char* file, std::vector<Particle>& wheels,
                              std::vector<Particle>& cage, std::vector<Spring>& springs,
                              Vector3 offset) {
    std::ifstream f(file); if (!f.is_open()) return;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream iss(line); std::string pre; iss >> pre;
        if (pre == "v") {
            Vector3 p; iss >> p.x >> p.y >> p.z;
            wheels.push_back({Add(p,offset), Add(p,offset), {0,0,0}, 4.0f, true});
        }
    }
    for (auto& w : wheels)
        for (size_t i = 0; i < cage.size(); i++)
            CreateSpring(springs, w, cage[i], 600.0f, 30.0f, true);
}

void LoadVisualSkin(const char* file, std::vector<VisualVertex>& verts,
                    std::vector<int>& idx, Vector3 offset) {
    std::ifstream f(file); if (!f.is_open()) return;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream iss(line); std::string pre; iss >> pre;
        if (pre == "v") {
            Vector3 p; iss >> p.x >> p.y >> p.z;
            verts.push_back({Add(p, offset), 0, {0,0,0}});
        } else if (pre == "f") {
            for (int i = 0; i < 3; i++) {
                std::string vd; iss >> vd;
                std::stringstream vs(vd.substr(0, vd.find('/')));
                int vi; vs >> vi; idx.push_back(vi - 1);
            }
        }
    }
}

void BindSkinToCage(std::vector<VisualVertex>& verts, const std::vector<Particle>& parts) {
    if (parts.empty()) return;
    float minZ=9999, maxZ=-9999, maxY=-9999;
    for (size_t i = 0; i < parts.size(); i++) {
        if (parts[i].position.z < minZ) { minZ=parts[i].position.z; pBack=i; }
        if (parts[i].position.z > maxZ) { maxZ=parts[i].position.z; pFront=i; }
        if (parts[i].position.y > maxY) { maxY=parts[i].position.y; pTop=i; }
    }
    Vector3 fwd = Normalize(Subtract(parts[pFront].position, parts[pBack].position));
    Vector3 tup = Normalize(Subtract(parts[pTop].position,  parts[pBack].position));
    Vector3 rgt = Normalize(Cross(tup, fwd));
    Vector3 up  = Cross(fwd, rgt);
    for (auto& vv : verts) {
        float md=999999; int ci=0;
        for (size_t i=0; i<parts.size(); i++) {
            float d=Distance(vv.position, parts[i].position);
            if (d < md) { md=d; ci=i; }
        }
        vv.boundParticleIndex = ci;
        Vector3 g = Subtract(vv.position, parts[ci].position);
        vv.offset = {Dot(g,rgt), Dot(g,up), Dot(g,fwd)};
    }
}

Obstacle LoadObstacle(const char* fileName, Vector3 position, Color color) {
    Obstacle obs;
    obs.position = position; obs.color = color;
    obs.model    = LoadModel(fileName); obs.hasModel = true;
    float minX=99999,minY=99999,minZ=99999, maxX=-99999,maxY=-99999,maxZ=-99999;
    Mesh& mesh = obs.model.meshes[0];
    for (int i = 0; i < mesh.vertexCount; i++) {
        float x = mesh.vertices[i*3+0]+position.x;
        float y = mesh.vertices[i*3+1]+position.y;
        float z = mesh.vertices[i*3+2]+position.z;
        if(x<minX)minX=x; if(x>maxX)maxX=x;
        if(y<minY)minY=y; if(y>maxY)maxY=y;
        if(z<minZ)minZ=z; if(z>maxZ)maxZ=z;
    }
    obs.minBounds={minX,minY,minZ}; obs.maxBounds={maxX,maxY,maxZ};
    return obs;
}

int main() {
    InitWindow(1280, 720, "Vortex Engine");
    SetTargetFPS(60);

    Camera3D camera = {0};
    camera.position={15,15,20}; camera.target={0,2,0};
    camera.up={0,1,0}; camera.fovy=45; camera.projection=CAMERA_PERSPECTIVE;

    std::vector<Particle>     cageParticles, wheels;
    std::vector<Spring>       allSprings;
    std::vector<VisualVertex> carSkin;
    std::vector<int>          carIndices;

    Vector3 spawnPoint = {0.0f, 3.0f, 5.0f};
    LoadPhysicsCage("cage.obj", cageParticles, allSprings, 180.0f, spawnPoint);
    LoadWheelsAndSuspension("wheels.obj", wheels, cageParticles, allSprings, spawnPoint);
    LoadVisualSkin("Car.obj", carSkin, carIndices, spawnPoint);
    BindSkinToCage(carSkin, cageParticles);

    Model wheelModels[4];
    wheelModels[0]=LoadModel("wheel_fl.obj"); wheelModels[1]=LoadModel("wheel_fr.obj");
    wheelModels[2]=LoadModel("wheel_rl.obj"); wheelModels[3]=LoadModel("wheel_rr.obj");

    std::vector<int> frontWheels, rearWheels;
    int fl=0, fr=1, rl=2, rr=3;
    if (wheels.size() == 4) {
        std::vector<std::pair<float,int>> zSort;
        for (int i=0;i<4;i++) zSort.push_back({wheels[i].position.z, i});
        std::sort(zSort.begin(), zSort.end());
        rl=zSort[0].second; rr=zSort[1].second;
        fl=zSort[2].second; fr=zSort[3].second;
        rearWheels.push_back(rl);  rearWheels.push_back(rr);
        frontWheels.push_back(fl); frontWheels.push_back(fr);
    }

    Obstacle pillar = LoadObstacle("pillar.obj", {0.0f,  0.0f, 20.0f}, GRAY);
    Obstacle floor  = LoadObstacle("floor.obj",  {0.0f,  0.0f,  0.0f}, LIGHTGRAY);

    float gravity=-15, wheelRadius=0.4f, maxEnginePower=2000;
    float currentGas=0, currentSteering=0, visualWheelRot=0;
    float camYaw=0, camPitch=0.4f, camDist=12;
    DisableCursor();

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.033f) dt = 0.033f;

        Vector2 md = GetMouseDelta();
        camYaw   -= md.x * 0.004f;
        camPitch -= md.y * 0.004f;
        camPitch  = Clamp(camPitch, 0.05f, 1.4f);
        camDist  -= GetMouseWheelMove() * 1.5f;
        camDist   = Clamp(camDist, 4.0f, 30.0f);

        Vector3 cBack      = cageParticles[pBack].position;
        Vector3 cFront     = cageParticles[pFront].position;
        Vector3 cTop       = cageParticles[pTop].position;
        Vector3 carForward = Normalize(Subtract(cFront, cBack));
        Vector3 carRight   = Normalize(Cross(Normalize(Subtract(cTop,cBack)), carForward));
        Vector3 carUp      = Cross(carForward, carRight);

        bool handbrake = IsKeyDown(KEY_SPACE);
        float tGas=0;
        if (IsKeyDown(KEY_W)) tGas= maxEnginePower;
        if (IsKeyDown(KEY_S)) tGas=-maxEnginePower;
        if (handbrake) tGas=0;
        currentGas += (tGas - currentGas) * 1.5f * dt;

        float tSteer=0;
        if (IsKeyDown(KEY_A)) tSteer=-0.6f;
        if (IsKeyDown(KEY_D)) tSteer= 0.6f;
        currentSteering += (tSteer - currentSteering) * 5.0f * dt;

        Vector3 fwdHeading = Add(Scale(carForward, std::cos(currentSteering)),
                                 Scale(carRight,  -std::sin(currentSteering)));

        Vector3 wheelStartPos[4];
        for (int i=0;i<4&&i<(int)wheels.size();i++) wheelStartPos[i]=wheels[i].position;

        int   subSteps = 10;
        float subDt    = dt / subSteps;

        for (int step = 0; step < subSteps; step++) {
            for (auto& p : cageParticles) p.acceleration.y += gravity;
            for (auto& w : wheels)        w.acceleration.y += gravity;
            for (auto& s : allSprings)    ApplySpringForce(s, subDt);

            for (auto& p : cageParticles) {
                UpdateParticle(p, subDt);
                if (p.position.y < 0.2f) {
                    p.position.y = 0.2f;
                    p.previous_position.y = p.position.y +
                        (p.position.y - p.previous_position.y) * 0.3f;
                }
                HandleObstacleCollision(p, pillar);
                HandleObstacleCollision(p, floor);
            }

            if (wheels.size() == 4) {
                for (int wIdx=0; wIdx<4; wIdx++) {
                    UpdateParticle(wheels[wIdx], subDt);

                    // Obstacles FIRST — must happen before ground snap
                    HandleObstacleCollision(wheels[wIdx], pillar);
                    HandleObstacleCollision(wheels[wIdx], floor);

                    // Ground snap — only if below ground (not overriding obstacle lift)
                    if (wheels[wIdx].position.y < wheelRadius) {
                        wheels[wIdx].position.y = wheelRadius;
                        wheels[wIdx].previous_position.y = wheels[wIdx].position.y;

                        Vector3 heading = (wIdx==frontWheels[0]||wIdx==frontWheels[1])
                                          ? fwdHeading : carForward;
                        float grip = 0.75f;
                        if (handbrake) {
                            grip = (wIdx==rearWheels[0]||wIdx==rearWheels[1]) ? 0.02f : 0.6f;
                            Vector3 vel = Subtract(wheels[wIdx].position, wheels[wIdx].previous_position);
                            float fwd   = Dot(vel, carForward);
                            wheels[wIdx].acceleration = Add(wheels[wIdx].acceleration,
                                                            Scale(carForward, -fwd*0.6f));
                        } else {
                            wheels[wIdx].acceleration = Add(wheels[wIdx].acceleration,
                                                            Scale(heading, currentGas));
                        }
                        ApplyTireFriction(wheels[wIdx], heading, grip);
                    }
                }
            }
        }

        if (wheels.size() == 4) {
            Vector3 vel = Subtract(wheels[rl].position, wheelStartPos[rl]);
            float distTraveled = Dot(vel, carForward);
            visualWheelRot += distTraveled / wheelRadius;
        }

        for (auto& vv : carSkin) {
            vv.position = Add(cageParticles[vv.boundParticleIndex].position,
                Add(Add(Scale(carRight, vv.offset.x),
                        Scale(carUp,    vv.offset.y)),
                        Scale(carForward,vv.offset.z)));
        }

        if (!cageParticles.empty()) {
            Vector3 cc={0,0,0};
            for (auto& p : cageParticles) { cc.x+=p.position.x; cc.y+=p.position.y; cc.z+=p.position.z; }
            float n=(float)cageParticles.size();
            cc.x/=n; cc.y/=n; cc.z/=n; cc.y+=1.2f;
            camera.target   = cc;
            camera.position = {cc.x + camDist*std::cos(camPitch)*std::sin(camYaw),
                               cc.y + camDist*std::sin(camPitch),
                               cc.z + camDist*std::cos(camPitch)*std::cos(camYaw)};
        }

        BeginDrawing();
            ClearBackground(SKYBLUE);
            BeginMode3D(camera);
                DrawGrid(40, 1.0f);

                if (pillar.hasModel)
                    DrawModel(pillar.model, pillar.position, 1.0f, pillar.color);
                else
                    DrawCube({(pillar.minBounds.x+pillar.maxBounds.x)/2.0f,
                               (pillar.minBounds.y+pillar.maxBounds.y)/2.0f,
                               (pillar.minBounds.z+pillar.maxBounds.z)/2.0f},
                              pillar.maxBounds.x-pillar.minBounds.x,
                              pillar.maxBounds.y-pillar.minBounds.y,
                              pillar.maxBounds.z-pillar.minBounds.z, pillar.color);

                if (floor.hasModel)
                    DrawModel(floor.model, floor.position, 1.0f, floor.color);

                rlBegin(RL_TRIANGLES);
                for (size_t i=0; i<carIndices.size(); i+=3) {
                    Vector3 v1=carSkin[carIndices[i]].position;
                    Vector3 v2=carSkin[carIndices[i+1]].position;
                    Vector3 v3=carSkin[carIndices[i+2]].position;
                    Vector3 n=Normalize(Cross(Subtract(v2,v1),Subtract(v3,v1)));
                    float l=std::abs(Dot(n,{0,1,0}))*0.5f+0.5f;
                    rlColor4ub(255*l,0,0,255);
                    rlVertex3f(v1.x,v1.y,v1.z); rlVertex3f(v2.x,v2.y,v2.z); rlVertex3f(v3.x,v3.y,v3.z);
                    rlVertex3f(v1.x,v1.y,v1.z); rlVertex3f(v3.x,v3.y,v3.z); rlVertex3f(v2.x,v2.y,v2.z);
                }
                rlEnd();

                rlBegin(RL_LINES);
                rlColor4ub(0,0,0,255);
                for (size_t i=0; i<carIndices.size(); i+=3) {
                    Vector3 v1=carSkin[carIndices[i]].position;
                    Vector3 v2=carSkin[carIndices[i+1]].position;
                    Vector3 v3=carSkin[carIndices[i+2]].position;
                    rlVertex3f(v1.x,v1.y,v1.z); rlVertex3f(v2.x,v2.y,v2.z);
                    rlVertex3f(v2.x,v2.y,v2.z); rlVertex3f(v3.x,v3.y,v3.z);
                    rlVertex3f(v3.x,v3.y,v3.z); rlVertex3f(v1.x,v1.y,v1.z);
                }
                rlEnd();

               auto DrawOneWheel = [&](Model& m, int physIdx, float steerAngle) {
                float carAngle = atan2(carForward.x, carForward.z);

                // m.transform only handles ROTATION (applied to local mesh vertices)
                // Translation is handled by DrawModel's position parameter
                Matrix spin    = MatrixRotateX(visualWheelRot);
                Matrix steer   = MatrixRotateY(steerAngle);
                Matrix body    = MatrixRotateY(carAngle);

                // In raylib: MatrixMultiply(A,B) = B*A (B applied first)
                // We want: body first, then steer, then spin
                m.transform = MatrixMultiply(spin, MatrixMultiply(steer, body));

                // Pass world position through DrawModel, NOT the matrix
                Vector3 wheelPos = wheels[physIdx].position;
                DrawModel(m, wheelPos, 1.0f, WHITE);
            };

                DrawOneWheel(wheelModels[0], fl, -currentSteering);
                DrawOneWheel(wheelModels[1], fr, -currentSteering);
                DrawOneWheel(wheelModels[2], rl, 0.0f);
                DrawOneWheel(wheelModels[3], rr, 0.0f);

            EndMode3D();
            DrawText("V2.3 CORE: Smooth Momentum + Batch Rendering + Real Wheels", 10, 10, 20, DARKGRAY);
            DrawText("WASD: Drive | SPACE: Handbrake Drift", 10, 40, 20, BLACK);
            DrawFPS(10, 70);
        EndDrawing();
    }

    for (int i=0;i<4;i++) UnloadModel(wheelModels[i]);
    UnloadModel(pillar.model);
    UnloadModel(floor.model);
    CloseWindow();
    return 0;
}