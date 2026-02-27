#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

// ─────────────────────────────────────────────
//  Math helpers
// ─────────────────────────────────────────────
Vector3 Subtract(Vector3 v1, Vector3 v2) { return {v1.x-v2.x, v1.y-v2.y, v1.z-v2.z}; }
Vector3 Add(Vector3 v1, Vector3 v2)      { return {v1.x+v2.x, v1.y+v2.y, v1.z+v2.z}; }
Vector3 Scale(Vector3 v, float s)        { return {v.x*s, v.y*s, v.z*s}; }
float   Dot(Vector3 v1, Vector3 v2)      { return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z; }
Vector3 Normalize(Vector3 v) {
    float l = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    return (l == 0.f) ? Vector3{0,0,0} : Vector3{v.x/l, v.y/l, v.z/l};
}
Vector3 Cross(Vector3 a, Vector3 b) {
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
float Distance(Vector3 a, Vector3 b) {
    Vector3 d = Subtract(a, b);
    return std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
}

// ─────────────────────────────────────────────
//  Particle / Spring
// ─────────────────────────────────────────────
struct Particle {
    Vector3 position, previous_position, acceleration;
    float   mass;
    bool    isWheel;
    // Visual damage accumulation (0..1) used for color tinting
    float   damage;
    Vector3 originalPosition;   // ← ADD THIS
};

struct Spring {
    Particle* p1;
    Particle* p2;

    float rest_length;      // current (possibly deformed) rest length
    float original_length;  // factory rest length

    float stiffness;        // k  — N/m equivalent
    float damping;          // c  — Ns/m equivalent

    bool  isSuspension;

    // ── Metal deformation parameters (BeamNG-inspired) ──
    // Force threshold before plastic yielding begins (N equivalent)
    float deformThreshold;
    // Maximum plastic deformation as fraction of original length
    float maxDeform;        // e.g. 0.55 → can crush up to 55 % of original
    // How much of excess force converts to permanent length change (0..1)
    float plasticRate;
    // Accumulated damage 0..1  (read-only; used for rendering tint)
    float damage;
    // Stiffness/damping after yield: softer and more lossy
    bool  hasYielded;
};

// ─────────────────────────────────────────────
//  Visual skin
// ─────────────────────────────────────────────
struct VisualVertex {
    Vector3 position;
    int   boundIndices[3];
    float weights[3];
    Vector3 localCoords[3];
    Vector3 deformedLocalCoords[3];   // ← ADD THIS
};

// ─────────────────────────────────────────────
//  Obstacles
// ─────────────────────────────────────────────
struct Obstacle {
    Vector3 minBounds, maxBounds, position;
    Color   color;
    Model   model;
    bool    hasModel;
};
struct SphereObstacle {
    Vector3 center, position;
    float   radius;
    Color   color;
    Model   model;
};

int pFront = 0, pBack = 0, pTop = 0;

// ─────────────────────────────────────────────
//  Particle integrator  (Verlet)
// ─────────────────────────────────────────────
void UpdateParticle(Particle& p, float dt) {
    // Linear damping slightly above your original (kills oscillation faster)
    Vector3 vel = Scale(Subtract(p.position, p.previous_position), 0.998f);
    p.previous_position = p.position;
    p.position.x += vel.x + p.acceleration.x * dt * dt;
    p.position.y += vel.y + p.acceleration.y * dt * dt;
    p.position.z += vel.z + p.acceleration.z * dt * dt;
    p.acceleration = {0,0,0};
}

void ApplyTireFriction(Particle& wheel, Vector3 heading, float grip) {
    Vector3 vel    = Subtract(wheel.position, wheel.previous_position);
    Vector3 fwdVel = Scale(heading, Dot(vel, heading));
    Vector3 latVel = Subtract(vel, fwdVel);
    float   retain = (1.f - grip) * (1.f - grip);
    wheel.previous_position = Subtract(wheel.position,
                                       Add(fwdVel, Scale(latVel, retain)));
}

// ─────────────────────────────────────────────
//  METAL SPRING FORCE  (BeamNG-style)
//
//  Key differences from your old version:
//   1. Very high stiffness (≈8 000 000 N/m in BeamNG; scaled down here to
//      ~4000-8000 since our world units are ~1 m and sub-steps are small).
//   2. Heavy damping (c ≈ 150-400) – real steel doesn't ring like a bell.
//   3. Plastic yielding: when the spring force magnitude exceeds
//      `deformThreshold`, the rest_length is nudged permanently toward the
//      current length.  After yield, stiffness is halved and damping doubled
//      to model the crumpled/softened zone (crushed metal).
//   4. A "hard floor" on compression – you cannot push through steel, so
//      compression below a minimum length causes a large restoring impulse.
// ─────────────────────────────────────────────
void ApplySpringForce(Spring& spring, float dt) {
    float dist = Distance(spring.p1->position, spring.p2->position);
    if (dist < 0.0001f) return;

    Vector3 dir          = Normalize(Subtract(spring.p2->position,
                                               spring.p1->position));
    float   displacement = dist - spring.rest_length;

    // Relative velocity along spring axis (for damping)
    Vector3 v1      = Subtract(spring.p1->position, spring.p1->previous_position);
    Vector3 v2      = Subtract(spring.p2->position, spring.p2->previous_position);
    float   relVel  = Dot(Subtract(v2, v1), dir);           // positive = stretching

    float   kEff = spring.stiffness;
    float   cEff = spring.damping;

    // ── Plastic yielding (permanent deformation) ──────────────────────────
    if (!spring.isSuspension) {
        float elasticForce = std::abs(kEff * displacement);
        if (elasticForce > spring.deformThreshold) {
            // Magnitude of excess force above yield threshold
            float excess    = elasticForce - spring.deformThreshold;
            // How much the rest length shifts permanently this sub-step
            float shift     = spring.plasticRate * (displacement / dist) *
                              (excess / spring.deformThreshold) * dt;

            float newRest   = spring.rest_length + shift;
            float minRest   = spring.original_length * (1.f - spring.maxDeform);
            float maxRest   = spring.original_length * (1.f + spring.maxDeform * 0.3f); // less stretch than crush

            spring.rest_length = Clamp(newRest, minRest, maxRest);

            // After first yield: soften & damp more (crumple zone behaviour)
            if (!spring.hasYielded) {
                spring.stiffness  *= 0.55f;   // crushed zone is softer
                spring.damping    *= 2.2f;    // crushed zone absorbs energy fast
                spring.hasYielded  = true;
            }
            // Track cumulative damage (visual)
            float deformFrac = std::abs(spring.rest_length - spring.original_length)
                               / (spring.original_length * spring.maxDeform + 0.0001f);
            spring.damage = Clamp(deformFrac, spring.damage, 1.f);

            // Push damage into particles too
            spring.p1->damage = Clamp(spring.p1->damage + deformFrac * 0.15f, 0.f, 1.f);
            spring.p2->damage = Clamp(spring.p2->damage + deformFrac * 0.15f, 0.f, 1.f);
        }
    }

    // Recompute after possible rest_length change
    displacement = dist - spring.rest_length;

    float dampForce   = relVel * cEff;
    float springForce = kEff * displacement + dampForce;

    // Hard-stop: cannot compress past (original_length * 0.35)
    if (!spring.isSuspension) {
        float minDist = spring.original_length * 0.35f;
        if (dist < minDist) {
            float penetration = minDist - dist;
            springForce -= penetration * spring.stiffness * 4.f;  // large repulsive jolt
        }
    }

    spring.p1->acceleration = Add(spring.p1->acceleration,
                                  Scale(dir,  springForce / spring.p1->mass));
    spring.p2->acceleration = Subtract(spring.p2->acceleration,
                                  Scale(dir,  springForce / spring.p2->mass));
}

// ─────────────────────────────────────────────
//  Obstacle collision
// ─────────────────────────────────────────────
void HandleObstacleCollision(Particle& p, const Obstacle& obs) {
    if (p.position.x <= obs.minBounds.x || p.position.x >= obs.maxBounds.x) return;
    if (p.position.y <= obs.minBounds.y || p.position.y >= obs.maxBounds.y) return;
    if (p.position.z <= obs.minBounds.z || p.position.z >= obs.maxBounds.z) return;
    float dX0=std::abs(p.position.x-obs.minBounds.x), dX1=std::abs(obs.maxBounds.x-p.position.x);
    float dY0=std::abs(p.position.y-obs.minBounds.y), dY1=std::abs(obs.maxBounds.y-p.position.y);
    float dZ0=std::abs(p.position.z-obs.minBounds.z), dZ1=std::abs(obs.maxBounds.z-p.position.z);
    float mn  = std::min({dX0,dX1,dY0,dY1,dZ0,dZ1});
    Vector3 vel = Subtract(p.position, p.previous_position);
    if      (mn==dX0){p.position.x=obs.minBounds.x; vel.x*=-0.01f; vel.y*=0.1f; vel.z*=0.1f;}
    else if (mn==dX1){p.position.x=obs.maxBounds.x; vel.x*=-0.01f; vel.y*=0.1f; vel.z*=0.1f;}
    else if (mn==dY0){p.position.y=obs.minBounds.y; vel.y*=-0.01f; vel.x*=0.1f; vel.z*=0.1f;}
    else if (mn==dY1){p.position.y=obs.maxBounds.y; vel.y*=-0.01f; vel.x*=0.1f; vel.z*=0.1f;}
    else if (mn==dZ0){p.position.z=obs.minBounds.z; vel.z*=-0.01f; vel.x*=0.1f; vel.y*=0.1f;}
    else             {p.position.z=obs.maxBounds.z; vel.z*=-0.01f; vel.x*=0.1f; vel.y*=0.1f;}
    vel.x*=0.5f; vel.y*=0.5f; vel.z*=0.5f;
    p.previous_position = Subtract(p.position, vel);
}

// ─────────────────────────────────────────────
//  Spring factory — NEW: takes deform params
// ─────────────────────────────────────────────
void CreateSpring(std::vector<Spring>& springs, Particle& p1, Particle& p2,
                  float k, float d, bool susp,
                  float deformThresh = 9999999.f,
                  float maxDeform    = 0.0f,
                  float plasticRate  = 0.0f) {
    float dist = Distance(p1.position, p2.position);
    springs.push_back({&p1, &p2, dist, dist, k, d, susp,
                       deformThresh, maxDeform, plasticRate, 0.f, false});
}

// ─────────────────────────────────────────────
//  CAGE LOADER
//  Physics parameters for the cage springs are set here.
//  Values inspired by BeamNG structural beams (scaled for our world).
//
//  k  = 6000   (very stiff — this is steel, not rubber)
//  d  = 350    (heavy damping — damps oscillations in < 0.5 s)
//  deformThreshold = 120 (force units; tune to taste)
//  maxDeform       = 0.55 (up to 55 % permanent crush)
//  plasticRate     = 0.35 (how fast the rest length moves on yield)
// ─────────────────────────────────────────────
void LoadPhysicsCage(const char* file, std::vector<Particle>& parts,
                     std::vector<Spring>& springs, float /*k_unused*/, Vector3 offset) {
    std::ifstream f(file); if (!f.is_open()) return;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream iss(line); std::string pre; iss >> pre;
        if (pre == "v") {
            Vector3 p; iss >> p.x >> p.y >> p.z;
            Vector3 wp = Add(p, offset);
            Particle newP;
            newP.position = wp;
            newP.previous_position = wp;
            newP.acceleration = {0,0,0};
            newP.mass = 2.0f;
            newP.isWheel = false;
            newP.damage = 0.f;
            newP.originalPosition = wp;
            parts.push_back(newP);
        }
    }
    for (size_t i = 0; i < parts.size(); i++)
        for (size_t j = i+1; j < parts.size(); j++)
            CreateSpring(springs, parts[i], parts[j],
                         6000.f,   // stiffness
                         350.f,    // damping  ← heavy, like steel
                         false,
                         800.f,    // yield threshold
                         0.55f,    // max crush ratio
                         0.35f);   // plastic rate
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
            wheels.push_back({Add(p,offset), Add(p,offset), {0,0,0}, 0.5f, true, 0.f});
        }
    }
    // Suspension: elastic, no permanent deform
    for (auto& w : wheels)
        for (size_t i = 0; i < cage.size(); i++)
            CreateSpring(springs, w, cage[i], 200.f, 60.f, true);
}

void LoadVisualSkin(const char* file, std::vector<VisualVertex>& verts,
                    std::vector<int>& idx, Vector3 offset) {
    std::ifstream f(file); if (!f.is_open()) return;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream iss(line); std::string pre; iss >> pre;
        if (pre == "v") {
            Vector3 p; iss >> p.x >> p.y >> p.z;
            VisualVertex vv; vv.position = Add(p, offset);
            for (int i=0;i<3;i++){vv.boundIndices[i]=0;vv.weights[i]=0.f;vv.localCoords[i]={0,0,0};}
            verts.push_back(vv);
        } else if (pre == "f") {
            for (int i=0;i<3;i++){
                std::string vd; iss >> vd;
                std::stringstream vs(vd.substr(0, vd.find('/')));
                int vi; vs >> vi; idx.push_back(vi-1);
            }
        }
    }
}

void BindSkinToCage(std::vector<VisualVertex>& verts, const std::vector<Particle>& parts) {
    if (parts.empty()) return;
    float minZ=9999, maxZ=-9999, maxY=-9999;
    for (size_t i=0;i<parts.size();i++){
        if (parts[i].position.z < minZ){minZ=parts[i].position.z; pBack=(int)i;}
        if (parts[i].position.z > maxZ){maxZ=parts[i].position.z; pFront=(int)i;}
        if (parts[i].position.y > maxY){maxY=parts[i].position.y; pTop=(int)i;}
    }
    Vector3 fwd = Normalize(Subtract(parts[pFront].position, parts[pBack].position));
    Vector3 tup = Normalize(Subtract(parts[pTop].position,   parts[pBack].position));
    Vector3 rgt = Normalize(Cross(tup, fwd));
    Vector3 up  = Cross(fwd, rgt);
    for (auto& vv : verts) {
        std::vector<std::pair<float,int>> di;
        for (size_t i=0;i<parts.size();i++) di.push_back({Distance(vv.position, parts[i].position),(int)i});
        std::sort(di.begin(), di.end());
        float total=0.f;
        for (int j=0;j<3;j++){float inv=1.f/(di[j].first+0.0001f); total+=inv; vv.boundIndices[j]=di[j].second; vv.weights[j]=inv;}
        for (int j=0;j<3;j++) vv.weights[j]/=total;
        for (int j=0;j<3;j++){
            int idx=vv.boundIndices[j];
            Vector3 g=Subtract(vv.position, parts[idx].position);
            vv.localCoords[j]={Dot(g,rgt), Dot(g,up), Dot(g,fwd)};
            vv.deformedLocalCoords[j] = vv.localCoords[j];   // ← ADD THIS
        }
    }
}

Obstacle LoadObstacle(const char* fileName, Vector3 position, Color color) {
    Obstacle obs; obs.position=position; obs.color=color;
    obs.model=LoadModel(fileName); obs.hasModel=true;
    float minX=99999,minY=99999,minZ=99999,maxX=-99999,maxY=-99999,maxZ=-99999;
    Mesh& mesh=obs.model.meshes[0];
    for(int i=0;i<mesh.vertexCount;i++){
        float x=mesh.vertices[i*3+0]+position.x;
        float y=mesh.vertices[i*3+1]+position.y;
        float z=mesh.vertices[i*3+2]+position.z;
        if(x<minX)minX=x;if(x>maxX)maxX=x;
        if(y<minY)minY=y;if(y>maxY)maxY=y;
        if(z<minZ)minZ=z;if(z>maxZ)maxZ=z;
    }
    obs.minBounds={minX,minY,minZ}; obs.maxBounds={maxX,maxY,maxZ};
    return obs;
}


void UpdateSkinDeformation(std::vector<VisualVertex>& verts,
                           const std::vector<Particle>& parts,
                           Vector3 rgt, Vector3 up, Vector3 fwd) {
    for (auto& vv : verts) {
        for (int j = 0; j < 3; j++)
            vv.deformedLocalCoords[j] = vv.localCoords[j];

        for (int j = 0; j < 3; j++) {
            int pi = vv.boundIndices[j];
            if (pi >= (int)parts.size()) continue;
            const Particle& p = parts[pi];
            if (p.damage < 0.01f) continue;

            // World-space displacement of this cage particle from its birth position
            Vector3 disp = Subtract(p.position, p.originalPosition);

            // Convert to local car space
            float dx = Dot(disp, rgt);
            float dy = Dot(disp, up);
            float dz = Dot(disp, fwd);

            // Pull the skin vertex toward the displaced cage particle
            float strength = Clamp(p.damage * 1.5f, 0.f, 1.f) * vv.weights[j];
            vv.deformedLocalCoords[j].x += dx * strength;
            vv.deformedLocalCoords[j].y += dy * strength;
            vv.deformedLocalCoords[j].z += dz * strength;
        }
    }
}

// ─────────────────────────────────────────────
//  MAIN
// ─────────────────────────────────────────────
int main() {
    InitWindow(1280, 720, "Vortex Engine — Metal Crash Physics");
    SetTargetFPS(60);

    Camera3D camera = {};
    camera.position={15,15,20}; camera.target={0,2,0};
    camera.up={0,1,0}; camera.fovy=45; camera.projection=CAMERA_PERSPECTIVE;

    std::vector<Particle>     cageParticles, wheels;
    std::vector<Spring>       allSprings;
    std::vector<VisualVertex> carSkin;
    std::vector<int>          carIndices;

    Vector3 spawnPoint = {0.f, 3.f, 5.f};
    LoadPhysicsCage("cage.obj",   cageParticles, allSprings, 900.f, spawnPoint);
    LoadWheelsAndSuspension("wheels.obj", wheels, cageParticles, allSprings, spawnPoint);
    LoadVisualSkin("Car.obj", carSkin, carIndices, spawnPoint);
    BindSkinToCage(carSkin, cageParticles);

    // ── Wheel anchor lateral constraint setup ──
    int   wheelAnchor[4];
    float wheelLocalRight[4], wheelLocalForward[4], wheelLocalUp0[4];
    {
        Vector3 cB = cageParticles[pBack].position;
        Vector3 cF = cageParticles[pFront].position;
        Vector3 cT = cageParticles[pTop].position;
        Vector3 fwd = Normalize(Subtract(cF,cB));
        Vector3 rgt = Normalize(Cross(Normalize(Subtract(cT,cB)), fwd));
        Vector3 upv = Cross(fwd, rgt);
        for (int wi=0;wi<4&&wi<(int)wheels.size();wi++){
            float best=999999.f; int bestIdx=0;
            for (int ci=0;ci<(int)cageParticles.size();ci++){
                float d=Distance(wheels[wi].position, cageParticles[ci].position);
                if(d<best){best=d; bestIdx=ci;}
            }
            wheelAnchor[wi]=bestIdx;
            Vector3 anchor=cageParticles[bestIdx].position;
            Vector3 off=Subtract(wheels[wi].position, anchor);
            wheelLocalRight[wi]  =Dot(off,rgt);
            wheelLocalForward[wi]=Dot(off,fwd);
            wheelLocalUp0[wi]    =Dot(off,upv);
        }
    }

    Model wheelModels[4];
    wheelModels[0]=LoadModel("wheel_fl.obj"); wheelModels[1]=LoadModel("wheel_fr.obj");
    wheelModels[2]=LoadModel("wheel_rl.obj"); wheelModels[3]=LoadModel("wheel_rr.obj");

    int fl=0,fr=1,rl=2,rr=3;
    std::vector<int> frontWheels, rearWheels;
    if (wheels.size()==4){
        std::vector<std::pair<float,int>> zs;
        for(int i=0;i<4;i++) zs.push_back({wheels[i].position.z,i});
        std::sort(zs.begin(),zs.end());
        rl=zs[0].second; rr=zs[1].second;
        fl=zs[2].second; fr=zs[3].second;
        rearWheels  = {rl,rr};
        frontWheels = {fl,fr};
    }

    Obstacle pillar = LoadObstacle("pillar.obj", {0.f,0.f,20.f}, GRAY);

    std::vector<SphereObstacle> bumps;
    const char* bumpFiles[]={
        "bump1.obj","bump2.obj","bump3.obj","bump4.obj","bump5.obj"};
    Color bumpColors[]={DARKGRAY,GRAY,DARKGRAY,GRAY,DARKGRAY};
    for(int b=0;b<5;b++){
        if(FileExists(bumpFiles[b])){
            Model m=LoadModel(bumpFiles[b]);
            Mesh& mesh=m.meshes[0];
            Vector3 cen={0,0,0};
            for(int i=0;i<mesh.vertexCount;i++){
                cen.x+=mesh.vertices[i*3+0];
                cen.y+=mesh.vertices[i*3+1];
                cen.z+=mesh.vertices[i*3+2];
            }
            cen.x/=mesh.vertexCount; cen.y/=mesh.vertexCount; cen.z/=mesh.vertexCount;
            float rad=0.f;
            for(int i=0;i<mesh.vertexCount;i++){
                float dx=mesh.vertices[i*3+0]-cen.x;
                float dy=mesh.vertices[i*3+1]-cen.y;
                float dz=mesh.vertices[i*3+2]-cen.z;
                float d=std::sqrt(dx*dx+dy*dy+dz*dz);
                if(d>rad) rad=d;
            }
            bumps.push_back({cen,{0,0,0},rad,bumpColors[b],m});
        }
    }

    // Physics constants
    const float gravity        = -15.f;
    const float wheelRadius    = 0.4f;
    const float maxEnginePower = 20000.f;

    float currentGas=0, currentSteering=0, visualWheelRot=0;
    float camYaw=0, camPitch=4.8f, camDist=8;
    DisableCursor();

    // ── Damage overlay accumulator ──
    float totalDamageVis = 0.f; // shown in HUD

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.033f) dt = 0.033f;

        // Camera
        Vector2 md = GetMouseDelta();
        camYaw   -= md.x * 0.004f;
        camPitch += md.y * 0.004f;
        camPitch  = Clamp(camPitch, 0.05f, 1.4f);
        camDist  -= GetMouseWheelMove() * 1.5f;
        camDist   = Clamp(camDist, 4.f, 30.f);

        // Car orientation (live)
        Vector3 cBack      = cageParticles[pBack].position;
        Vector3 cFront     = cageParticles[pFront].position;
        Vector3 cTop       = cageParticles[pTop].position;
        Vector3 carForward = Normalize(Subtract(cFront, cBack));
        Vector3 carRight   = Normalize(Cross(Normalize(Subtract(cTop,cBack)), carForward));
        Vector3 carUp      = Cross(carForward, carRight);

        // Input
        bool handbrake = IsKeyDown(KEY_SPACE);
        float tGas = 0;
        if (IsKeyDown(KEY_W)) tGas =  maxEnginePower;
        if (IsKeyDown(KEY_S)) tGas = -maxEnginePower;
        if (handbrake) tGas = 0;
        currentGas += (tGas - currentGas) * 1.5f * dt;

        float tSteer = 0;
        if (IsKeyDown(KEY_A)) tSteer = -0.6f;
        if (IsKeyDown(KEY_D)) tSteer =  0.6f;
        currentSteering += (tSteer - currentSteering) * 5.f * dt;

        Vector3 fwdHeading = Add(Scale(carForward,  std::cos(currentSteering)),
                                 Scale(carRight,    -std::sin(currentSteering)));

        Vector3 wheelStartPos[4];
        for(int i=0;i<4&&i<(int)wheels.size();i++) wheelStartPos[i]=wheels[i].position;

        // ── Sub-step integration (20 steps → stable high-k springs) ──
        const int subSteps = 20;
        const float subDt  = dt / subSteps;

        for (int step=0; step<subSteps; step++) {
            for (auto& p : cageParticles) p.acceleration.y += gravity;
            for (auto& w : wheels)        w.acceleration.y += gravity;

            for (auto& s : allSprings) {
                ApplySpringForce(s, subDt);
                if (s.isSuspension) {
                    float dist   = Distance(s.p1->position, s.p2->position);
                    float maxLen = s.original_length * 1.4f;
                    float minLen = s.original_length * 0.5f;
                    if (dist > maxLen || dist < minLen) {
                        float target = (dist > maxLen) ? maxLen : minLen;
                        Vector3 dir  = Normalize(Subtract(s.p2->position, s.p1->position));
                        s.p1->position = Add(s.p1->position, Scale(dir, dist - target));
                    }
                }
            }

            // Cage particles
            for (auto& p : cageParticles) {
                UpdateParticle(p, subDt);
                if (p.position.y < 0.3f) {
                    Vector3 vel = Subtract(p.position, p.previous_position);
                    if (vel.y < 0) vel.y *= -0.1f; else vel.y=0;
                    p.position.y = 0.3f;
                    vel.x *= 0.995f; vel.z *= 0.995f;
                    p.previous_position = Subtract(p.position, vel);
                }
                HandleObstacleCollision(p, pillar);
                for (auto& bump : bumps) {
                    Vector3 dx = Subtract(p.position, bump.center);
                    float   d  = std::sqrt(dx.x*dx.x + dx.y*dx.y + dx.z*dx.z);
                    float   mn = bump.radius + 0.15f;
                    if (d < mn && d > 0.001f) {
                        float nx=dx.x/d, ny=dx.y/d, nz=dx.z/d;
                        float ov=mn-d;
                        Vector3 vel=Subtract(p.position, p.previous_position);
                        float vn=vel.x*nx+vel.y*ny+vel.z*nz;
                        if(vn<0){vel.x-=vn*nx; vel.y-=vn*ny; vel.z-=vn*nz;}
                        p.position.x+=nx*ov; p.position.y+=ny*ov; p.position.z+=nz*ov;
                        p.previous_position = Subtract(p.position, vel);
                    }
                }
            }

            // Wheels
            if (wheels.size()==4) {
                for (int wIdx=0;wIdx<4;wIdx++) {
                    UpdateParticle(wheels[wIdx], subDt);
                    HandleObstacleCollision(wheels[wIdx], pillar);

                    bool onBump=false;
                    for (auto& bump : bumps) {
                        Vector3 dx=Subtract(wheels[wIdx].position, bump.center);
                        float d=std::sqrt(dx.x*dx.x+dx.y*dx.y+dx.z*dx.z);
                        float mn=bump.radius+wheelRadius;
                        if(d<mn&&d>0.001f){
                            float nx=dx.x/d,ny=dx.y/d,nz=dx.z/d, ov=mn-d;
                            Vector3 vel=Subtract(wheels[wIdx].position, wheels[wIdx].previous_position);
                            float vn=vel.x*nx+vel.y*ny+vel.z*nz;
                            if(vn<0){vel.x-=vn*nx;vel.y-=vn*ny;vel.z-=vn*nz;}
                            wheels[wIdx].position.x+=nx*ov;
                            wheels[wIdx].position.y+=ny*ov;
                            wheels[wIdx].position.z+=nz*ov;
                            wheels[wIdx].previous_position=Subtract(wheels[wIdx].position,vel);
                            if(ny>0.3f) onBump=true;
                        }
                    }

                    if (!onBump && wheels[wIdx].position.y < wheelRadius){
                        Vector3 vel=Subtract(wheels[wIdx].position, wheels[wIdx].previous_position);
                        if(vel.y<0) vel.y*=-0.2f; else vel.y=0;
                        wheels[wIdx].position.y=wheelRadius;
                        wheels[wIdx].previous_position=Subtract(wheels[wIdx].position,vel);
                    }

                    bool grounded = onBump || (wheels[wIdx].position.y <= wheelRadius+0.15f);
                    if (grounded) {
                        Vector3 heading=(wIdx==frontWheels[0]||wIdx==frontWheels[1]) ? fwdHeading : carForward;
                        float grip=0.75f;
                        if(handbrake){
                            grip=(wIdx==rearWheels[0]||wIdx==rearWheels[1])?0.02f:0.6f;
                            Vector3 vel=Subtract(wheels[wIdx].position, wheels[wIdx].previous_position);
                            float fwd=Dot(vel, carForward);
                            wheels[wIdx].acceleration=Add(wheels[wIdx].acceleration, Scale(carForward,-fwd*0.6f));
                        } else {
                            wheels[wIdx].acceleration=Add(wheels[wIdx].acceleration, Scale(heading, currentGas));
                        }
                        ApplyTireFriction(wheels[wIdx], heading, grip);
                    }
                }
            }
        } // end sub-steps

        // Lateral wheel constraint
        for (int wIdx=0;wIdx<4;wIdx++){
            Vector3 anchor  = cageParticles[wheelAnchor[wIdx]].position;
            Vector3 toWheel = Subtract(wheels[wIdx].position, anchor);
            float   suspOff = Dot(toWheel, carUp);
            Vector3 desired = Add(anchor, Add(Scale(carRight,   wheelLocalRight[wIdx]),
                                        Add(Scale(carForward, wheelLocalForward[wIdx]),
                                            Scale(carUp,     suspOff))));
            Vector3 delta = Subtract(desired, wheels[wIdx].position);
            wheels[wIdx].position = desired;
            wheels[wIdx].previous_position = Add(wheels[wIdx].previous_position, delta);
        }

        // Visual wheel rotation
        if (wheels.size()==4){
            float spd=0.f;
            for(int i=0;i<4;i++) spd+=Dot(Subtract(wheels[i].position, wheelStartPos[i]), carForward);
            visualWheelRot+=(spd*0.25f)/wheelRadius;
        }

        // Update skin
        UpdateSkinDeformation(carSkin, cageParticles, carRight, carUp, carForward);
          for (auto& vv : carSkin){
            Vector3 np={0,0,0};
            for(int j=0;j<3;j++){
                int idx=vv.boundIndices[j];
                Vector3 off=Add(Add(Scale(carRight,   vv.deformedLocalCoords[j].x),
                     Scale(carUp,      vv.deformedLocalCoords[j].y)),
                     Scale(carForward, vv.deformedLocalCoords[j].z));
                np=Add(np, Scale(Add(cageParticles[idx].position, off), vv.weights[j]));
            }
            vv.position=np;
        }

        // Accumulate total damage for HUD
        totalDamageVis = 0.f;
        for (auto& p : cageParticles) totalDamageVis = std::max(totalDamageVis, p.damage);

        // Camera follow
        if (!cageParticles.empty()){
            Vector3 cc={0,0,0};
            for(auto& p:cageParticles){cc.x+=p.position.x;cc.y+=p.position.y;cc.z+=p.position.z;}
            float n=(float)cageParticles.size();
            cc.x/=n; cc.y/=n; cc.z/=n; cc.y+=1.2f;
            camera.target=cc;
            camera.position={cc.x+camDist*std::cos(camPitch)*std::sin(camYaw),
                             cc.y+camDist*std::sin(camPitch),
                             cc.z+camDist*std::cos(camPitch)*std::cos(camYaw)};
        }

        // ─────────────────────── RENDER ───────────────────────
        BeginDrawing();
        ClearBackground(SKYBLUE);
        BeginMode3D(camera);
            DrawGrid(40, 1.f);
            DrawPlane({0,0,0},{200,200}, LIGHTGRAY);

            if (pillar.hasModel) DrawModel(pillar.model, pillar.position, 1.f, pillar.color);
            for (auto& bump : bumps) DrawModel(bump.model, bump.position, 1.f, bump.color);

            // ── Car body: colour shifts from red→dark-gray as damage accumulates ──
            rlBegin(RL_TRIANGLES);
            for (size_t i=0; i<carIndices.size(); i+=3){
                if (carIndices[i]   >= (int)carSkin.size() ||
                    carIndices[i+1] >= (int)carSkin.size() ||
                    carIndices[i+2] >= (int)carSkin.size()) continue;

                Vector3 v1=carSkin[carIndices[i]  ].position;
                Vector3 v2=carSkin[carIndices[i+1]].position;
                Vector3 v3=carSkin[carIndices[i+2]].position;
                Vector3 n =Normalize(Cross(Subtract(v2,v1),Subtract(v3,v1)));
                float   lit=std::abs(Dot(n,{0,1,0}))*0.5f+0.5f;

                // Local damage from nearest cage particles (cheap: use bound[0])
                float dmg = 0.f;
                for (int k=0;k<3;k++){
                    int bi = carSkin[carIndices[i+k]].boundIndices[0];
                    if (bi < (int)cageParticles.size())
                        dmg = std::max(dmg, cageParticles[bi].damage);
                }
                // Interpolate colour: red (clean) → dark-gray (destroyed)
                float rc = Clamp(1.f-dmg*0.8f, 0.2f, 1.f);
                float gc = Clamp(dmg*0.1f,      0.f,  0.1f);
                float bc = Clamp(dmg*0.1f,      0.f,  0.1f);
                unsigned char R=(unsigned char)(255*lit*rc);
                unsigned char G=(unsigned char)(255*lit*gc);
                unsigned char B=(unsigned char)(255*lit*bc);

                rlColor4ub(R,G,B,255);
                rlVertex3f(v1.x,v1.y,v1.z);
                rlVertex3f(v2.x,v2.y,v2.z);
                rlVertex3f(v3.x,v3.y,v3.z);
                rlVertex3f(v1.x,v1.y,v1.z);
                rlVertex3f(v3.x,v3.y,v3.z);
                rlVertex3f(v2.x,v2.y,v2.z);
            }
            rlEnd();

            rlBegin(RL_LINES);
            rlColor4ub(0,0,0,180);
            for (size_t i=0;i<carIndices.size();i+=3){
                if (carIndices[i]>=(int)carSkin.size()||
                    carIndices[i+1]>=(int)carSkin.size()||
                    carIndices[i+2]>=(int)carSkin.size()) continue;
                Vector3 v1=carSkin[carIndices[i]  ].position;
                Vector3 v2=carSkin[carIndices[i+1]].position;
                Vector3 v3=carSkin[carIndices[i+2]].position;
                rlVertex3f(v1.x,v1.y,v1.z); rlVertex3f(v2.x,v2.y,v2.z);
                rlVertex3f(v2.x,v2.y,v2.z); rlVertex3f(v3.x,v3.y,v3.z);
                rlVertex3f(v3.x,v3.y,v3.z); rlVertex3f(v1.x,v1.y,v1.z);
            }
            rlEnd();

            // Wheels
            auto DrawOneWheel = [&](Model& m, int physIdx, float angle){
                Matrix matScale = MatrixScale(1.f,1.f,1.f);
                Matrix matTrans = MatrixTranslate(wheels[physIdx].position.x,
                                                  wheels[physIdx].position.y,
                                                  wheels[physIdx].position.z);
                Matrix matRot   = MatrixMultiply(MatrixRotateX(visualWheelRot),
                                                 MatrixRotateY(angle));
                Matrix matBody  = {0};
                matBody.m0 =carRight.x;   matBody.m1 =carRight.y;   matBody.m2 =carRight.z;   matBody.m3 =0;
                matBody.m4 =carUp.x;      matBody.m5 =carUp.y;      matBody.m6 =carUp.z;      matBody.m7 =0;
                matBody.m8 =carForward.x; matBody.m9 =carForward.y; matBody.m10=carForward.z; matBody.m11=0;
                matBody.m12=0;            matBody.m13=0;             matBody.m14=0;            matBody.m15=1;
                m.transform=MatrixMultiply(matScale,
                            MatrixMultiply(matRot,
                            MatrixMultiply(matBody, matTrans)));
                DrawModel(m,{0,0,0},1.f,WHITE);
            };
            DrawOneWheel(wheelModels[0], fl, -currentSteering);
            DrawOneWheel(wheelModels[1], fr, -currentSteering);
            DrawOneWheel(wheelModels[2], rl,  0.f);
            DrawOneWheel(wheelModels[3], rr,  0.f);

        EndMode3D();

        // ── HUD ──
        DrawText("VORTEX ENGINE — METAL CRASH PHYSICS", 10, 10, 18, DARKGRAY);
        DrawText("WASD: Drive | SPACE: Handbrake", 10, 34, 16, BLACK);
        DrawFPS(10, 58);

        // Damage bar
        {
            int barW = 300, barH = 18;
            int barX = 1280 - barW - 10, barY = 10;
            DrawRectangle(barX-2, barY-2, barW+4, barH+4, BLACK);
            DrawRectangle(barX, barY, barW, barH, DARKGRAY);
            int fillW = (int)(barW * totalDamageVis);
            Color dmgColor = {(unsigned char)(255*(totalDamageVis)),
                              (unsigned char)(255*(1.f-totalDamageVis)), 0, 255};
            DrawRectangle(barX, barY, fillW, barH, dmgColor);
            DrawText("DAMAGE", barX+4, barY+2, 14, WHITE);
        }

        EndDrawing();
    }

    for(int i=0;i<4;i++) UnloadModel(wheelModels[i]);
    UnloadModel(pillar.model);
    for(auto& b:bumps) UnloadModel(b.model);
    CloseWindow();
    return 0;
}