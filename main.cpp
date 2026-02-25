#include "raylib.h"
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
};

struct Spring {
    Particle* p1;
    Particle* p2;
    float rest_length;
    float stiffness;
    float damping;
};

struct Obstacle {
    Vector3 minBounds;
    Vector3 maxBounds;
    Color color;
};

// ==========================================
// 2. MATH & PHYSICS BACKEND
// ==========================================

float Vector3Distance(Vector3 v1, Vector3 v2) {
    float dx = v2.x - v1.x;
    float dy = v2.y - v1.y;
    float dz = v2.z - v1.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void UpdateParticle(Particle& p, float deltaTime) {
    Vector3 velocity = {
        p.position.x - p.previous_position.x,
        p.position.y - p.previous_position.y,
        p.position.z - p.previous_position.z
    };

    float friction = (p.position.y <= 0.25f) ? 0.90f : 0.99f;
    velocity.x *= friction;
    velocity.y *= friction;
    velocity.z *= friction;

    p.previous_position = p.position;

    p.position.x += velocity.x + p.acceleration.x * deltaTime * deltaTime;
    p.position.y += velocity.y + p.acceleration.y * deltaTime * deltaTime;
    p.position.z += velocity.z + p.acceleration.z * deltaTime * deltaTime;

    p.acceleration = {0.0f, 0.0f, 0.0f};
}

void ApplySpringForce(Spring& spring) {
    float current_distance = Vector3Distance(spring.p1->position, spring.p2->position);
    if (current_distance == 0.0f) return;

    float displacement = current_distance - spring.rest_length;
    float force = spring.stiffness * displacement;

    Vector3 direction = {
        (spring.p2->position.x - spring.p1->position.x) / current_distance,
        (spring.p2->position.y - spring.p1->position.y) / current_distance,
        (spring.p2->position.z - spring.p1->position.z) / current_distance
    };

    float acc1 = force / spring.p1->mass;
    float acc2 = force / spring.p2->mass;

    spring.p1->acceleration.x += direction.x * acc1;
    spring.p1->acceleration.y += direction.y * acc1;
    spring.p1->acceleration.z += direction.z * acc1;

    spring.p2->acceleration.x -= direction.x * acc2;
    spring.p2->acceleration.y -= direction.y * acc2;
    spring.p2->acceleration.z -= direction.z * acc2;
}

void HandleObstacleCollision(Particle& p, const Obstacle& obs) {
    if (p.position.x > obs.minBounds.x && p.position.x < obs.maxBounds.x &&
        p.position.y > obs.minBounds.y && p.position.y < obs.maxBounds.y &&
        p.position.z > obs.minBounds.z && p.position.z < obs.maxBounds.z) {
        
        float distToMinX = p.position.x - obs.minBounds.x;
        float distToMaxX = obs.maxBounds.x - p.position.x;
        float distToMinY = p.position.y - obs.minBounds.y;
        float distToMaxY = obs.maxBounds.y - p.position.y;
        float distToMinZ = p.position.z - obs.minBounds.z;
        float distToMaxZ = obs.maxBounds.z - p.position.z;

        float minDist = std::min({distToMinX, distToMaxX, distToMinY, distToMaxY, distToMinZ, distToMaxZ});

        if (minDist == distToMinX) { p.position.x = obs.minBounds.x; p.previous_position.x = p.position.x; }
        else if (minDist == distToMaxX) { p.position.x = obs.maxBounds.x; p.previous_position.x = p.position.x; }
        else if (minDist == distToMinY) { p.position.y = obs.minBounds.y; p.previous_position.y = p.position.y; }
        else if (minDist == distToMaxY) { p.position.y = obs.maxBounds.y; p.previous_position.y = p.position.y; }
        else if (minDist == distToMinZ) { p.position.z = obs.minBounds.z; p.previous_position.z = p.position.z; }
        else if (minDist == distToMaxZ) { p.position.z = obs.maxBounds.z; p.previous_position.z = p.position.z; }
    }
}

// ==========================================
// 3. OBJ PARSER & SCENE GENERATION
// ==========================================

void CreateSpring(std::vector<Spring>& springs, Particle& p1, Particle& p2, float stiffness) {
    float dist = Vector3Distance(p1.position, p2.position);
    springs.push_back({&p1, &p2, dist, stiffness, 0.1f});
}


// Bypasses Raylib's visual loader and reads the raw OBJ text file directly.
void LoadMeshIntoPhysics(const char* fileName, std::vector<Particle>& particles, std::vector<Spring>& springs, std::vector<int>& indices, float stiffness, Vector3 spawnOffset) {
    std::ifstream file(fileName);
    if (!file.is_open()) {
        DrawText("Failed to open OBJ file!", 10, 50, 20, RED);
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            // It's a Vertex! Add it to our particles array.
            Vector3 pos;
            iss >> pos.x >> pos.y >> pos.z;
            pos.x += spawnOffset.x;
            pos.y += spawnOffset.y;
            pos.z += spawnOffset.z;
            particles.push_back({pos, pos, {0.0f, 0.0f, 0.0f}, 1.0f});
        } 
        else if (prefix == "f") {
            // It's a Face! Connect the vertices with springs.
            int v[3];
            for (int i = 0; i < 3; i++) {
                std::string vertexData;
                iss >> vertexData;
                
                // OBJ faces often look like "1/1/1". We only want the first number (the vertex index).
                std::stringstream vStream(vertexData.substr(0, vertexData.find('/')));
                vStream >> v[i];
                
                // OBJ files start counting at 1, but C++ vectors start at 0. So we subtract 1.
                v[i] -= 1; 
            }

            // Save for the visual renderer
            indices.push_back(v[0]);
            indices.push_back(v[1]);
            indices.push_back(v[2]);

            // Build the structural springs to hold the face together
            CreateSpring(springs, particles[v[0]], particles[v[1]], stiffness);
            CreateSpring(springs, particles[v[1]], particles[v[2]], stiffness);
            CreateSpring(springs, particles[v[2]], particles[v[0]], stiffness);
        }
    }
    file.close();
}







// ==========================================
// 4. MAIN ENGINE LOOP
// ==========================================
int main() {
    InitWindow(1024, 768, "Vortex Engine V0.5 - Custom OBJ Loader");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = { 15.0f, 15.0f, 20.0f };
    camera.target = { 0.0f, 2.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    std::vector<Particle> p;
    std::vector<Spring> springs;
    std::vector<int> renderIndices;

    float stiffness = 800.0f; // Increased stiffness so the custom model holds its shape better
    
    // Load your custom Blender file here! Spawning it 5 units in the air.
    LoadMeshIntoPhysics("Cuboid.obj", p, springs, renderIndices, stiffness, {0.0f, 5.0f, 0.0f});

    Obstacle pillar = { {-5.0f, 0.0f, -5.0f}, {-2.0f, 10.0f, -2.0f}, GRAY };

    float gravity = -15.0f; 
    float driveSpeed = 50.0f;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.033f) dt = 0.033f; 

        // 1. Handle WASD Input
        Vector3 driveForce = {0.0f, 0.0f, 0.0f};
        if (IsKeyDown(KEY_W)) driveForce.z -= driveSpeed;
        if (IsKeyDown(KEY_S)) driveForce.z += driveSpeed;
        if (IsKeyDown(KEY_A)) driveForce.x -= driveSpeed;
        if (IsKeyDown(KEY_D)) driveForce.x += driveSpeed;

        // Apply external forces
        for (auto& particle : p) {
            particle.acceleration.y += gravity;
            // Applying driving force to all particles dynamically for testing
            particle.acceleration.x += driveForce.x;
            particle.acceleration.z += driveForce.z;
        }

        // 2. Apply Springs
        for (auto& s : springs) ApplySpringForce(s);

        // 3. Integrate & Handle Collisions
        for (auto& particle : p) {
            UpdateParticle(particle, dt);

            // Floor Collision
            if (particle.position.y < 0.2f) {
                particle.position.y = 0.2f;
                float vy = particle.position.y - particle.previous_position.y;
                particle.previous_position.y = particle.position.y + (vy * 0.3f); 
            }

            HandleObstacleCollision(particle, pillar);
        }

        // Camera follow (locks onto the first vertex loaded)
        if (!p.empty()) {
            camera.target = p[0].position;
            camera.position = { p[0].position.x + 10.0f, 10.0f, p[0].position.z + 15.0f };
        }

        // --- RENDER ---
        BeginDrawing();
            ClearBackground(SKYBLUE);
            
            BeginMode3D(camera);
                DrawGrid(40, 1.0f);
                
                // Draw Obstacle
                DrawCube({(pillar.minBounds.x + pillar.maxBounds.x)/2, (pillar.minBounds.y + pillar.maxBounds.y)/2, (pillar.minBounds.z + pillar.maxBounds.z)/2}, 
                         pillar.maxBounds.x - pillar.minBounds.x, pillar.maxBounds.y - pillar.minBounds.y, pillar.maxBounds.z - pillar.minBounds.z, pillar.color);

                // Draw Dynamic Polygons from OBJ
                for (size_t i = 0; i < renderIndices.size(); i += 3) {
                    Vector3 v1 = p[renderIndices[i]].position;
                    Vector3 v2 = p[renderIndices[i+1]].position;
                    Vector3 v3 = p[renderIndices[i+2]].position;

                    // Draw double-sided triangles
                    DrawTriangle3D(v1, v2, v3, RED);
                    DrawTriangle3D(v3, v2, v1, RED);
                }

                // Draw Wireframe overlay
                for (auto& s : springs) {
                    DrawLine3D(s.p1->position, s.p2->position, BLACK);
                }

            EndMode3D();

            DrawText("Custom OBJ Loaded! WASD to Drive", 10, 10, 20, DARKGRAY);
            DrawFPS(10, 40);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}