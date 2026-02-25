#include "raylib.h"
#include <cmath>
#include <vector>
#include <algorithm>

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

// Loads the OBJ, welds duplicate vertices, and generates internal volume springs
void LoadMeshIntoPhysics(const char* fileName, std::vector<Particle>& particles, std::vector<Spring>& springs, std::vector<int>& indices, float stiffness, Vector3 spawnOffset) {
    Model model = LoadModel(fileName);
    if (model.meshes == NULL || model.meshCount == 0) {
        return; 
    }

    Mesh mesh = model.meshes[0]; 

    // --- 1. THE WELDER (Extract Unique Vertices) ---
    // We keep a map to translate Blender's bloated vertex list to our optimized, welded physics list
    std::vector<int> originalToUniqueMap(mesh.vertexCount);

    for (int i = 0; i < mesh.vertexCount; i++) {
        Vector3 pos = {
            mesh.vertices[i * 3] + spawnOffset.x,
            mesh.vertices[i * 3 + 1] + spawnOffset.y,
            mesh.vertices[i * 3 + 2] + spawnOffset.z
        };

        // Check if a particle already exists at this exact spot (welding distance: 0.001)
        int existingIndex = -1;
        for (size_t j = 0; j < particles.size(); j++) {
            if (Vector3Distance(particles[j].position, pos) < 0.001f) {
                existingIndex = j;
                break;
            }
        }

        if (existingIndex == -1) {
            // It's a brand new corner. Add it to our physics engine.
            particles.push_back({pos, pos, {0.0f, 0.0f, 0.0f}, 1.0f});
            originalToUniqueMap[i] = particles.size() - 1;
        } else {
            // Duplicate found! Weld it to the existing particle.
            originalToUniqueMap[i] = existingIndex;
        }
    }

    // --- 2. THE SHELL (Extract Triangles using our Welded Vertices) ---
    if (mesh.indices != NULL) {
        for (int i = 0; i < mesh.triangleCount; i++) {
            // Grab the indices, but run them through our map so they share the welded corners
            int idx1 = originalToUniqueMap[mesh.indices[i * 3]];
            int idx2 = originalToUniqueMap[mesh.indices[i * 3 + 1]];
            int idx3 = originalToUniqueMap[mesh.indices[i * 3 + 2]];

            indices.push_back(idx1); indices.push_back(idx2); indices.push_back(idx3);

            CreateSpring(springs, particles[idx1], particles[idx2], stiffness);
            CreateSpring(springs, particles[idx2], particles[idx3], stiffness);
            CreateSpring(springs, particles[idx3], particles[idx1], stiffness);
        }
    } else {
        // Handle unrolled meshes using the same welded map
        for (int i = 0; i < mesh.vertexCount; i += 3) {
            int idx1 = originalToUniqueMap[i];
            int idx2 = originalToUniqueMap[i + 1];
            int idx3 = originalToUniqueMap[i + 2];

            indices.push_back(idx1); indices.push_back(idx2); indices.push_back(idx3);

            CreateSpring(springs, particles[idx1], particles[idx2], stiffness);
            CreateSpring(springs, particles[idx2], particles[idx3], stiffness);
            CreateSpring(springs, particles[idx3], particles[idx1], stiffness);
        }
    }

    // --- 3. THE BRACER (Generate Internal Structure) ---
    // Connect particles across the inside of the shape so it doesn't fold flat
    for (size_t i = 0; i < particles.size(); i++) {
        for (size_t j = i + 1; j < particles.size(); j++) {
            float dist = Vector3Distance(particles[i].position, particles[j].position);
            // If they are within a certain distance, shoot a structural spring between them
            // We use 5.0f here assuming your cuboid isn't massive. 
            if (dist > 0.1f && dist < 5.0f) {
                // We make internal springs slightly softer so the outside shell takes the impact
                CreateSpring(springs, particles[i], particles[j], stiffness * 0.8f); 
            }
        }
    }

    UnloadModel(model); 
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