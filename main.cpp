#include "raylib.h"
#include <cmath>
#include <vector>

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

// Define a rigid obstacle
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

    // Ground friction: If it's touching the ground, slow it down so it doesn't slide like ice
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

// AABB Collision: Checks if a particle is inside a box and pushes it out
void HandleObstacleCollision(Particle& p, const Obstacle& obs) {
    if (p.position.x > obs.minBounds.x && p.position.x < obs.maxBounds.x &&
        p.position.y > obs.minBounds.y && p.position.y < obs.maxBounds.y &&
        p.position.z > obs.minBounds.z && p.position.z < obs.maxBounds.z) {
        
        // Find the closest face of the box to push the particle out of
        float distToMinX = p.position.x - obs.minBounds.x;
        float distToMaxX = obs.maxBounds.x - p.position.x;
        float distToMinY = p.position.y - obs.minBounds.y;
        float distToMaxY = obs.maxBounds.y - p.position.y;
        float distToMinZ = p.position.z - obs.minBounds.z;
        float distToMaxZ = obs.maxBounds.z - p.position.z;

        // Push out on the shortest axis and kill momentum on that axis to simulate a crash
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
// 3. RENDER FRONTEND
// ==========================================

void CreateSpring(std::vector<Spring>& springs, Particle& p1, Particle& p2, float stiffness) {
    float dist = Vector3Distance(p1.position, p2.position);
    springs.push_back({&p1, &p2, dist, stiffness, 0.1f});
}

// Helper to draw a solid quad (2 triangles) between 4 particles
void DrawSoftFace(Particle& p1, Particle& p2, Particle& p3, Particle& p4, Color color) {
    // Draw double-sided triangles so they don't disappear when the car crumples
    DrawTriangle3D(p1.position, p2.position, p3.position, color);
    DrawTriangle3D(p3.position, p2.position, p1.position, color);
    
    DrawTriangle3D(p1.position, p3.position, p4.position, color);
    DrawTriangle3D(p4.position, p3.position, p1.position, color);
}

// ==========================================
// 4. MAIN ENGINE LOOP
// ==========================================
int main() {
    InitWindow(1024, 768, "Vortex Engine V0.4 - Drivable Polygons");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = { 15.0f, 15.0f, 20.0f };
    camera.target = { 0.0f, 2.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    std::vector<Particle> p;
    std::vector<Spring> springs;

    float startY = 1.0f;
    float size = 2.0f;
    
    // Bottom 4 vertices
    p.push_back({{0.0f, startY, 0.0f}, {0.0f, startY, 0.0f}, {0.0f,0.0f,0.0f}, 1.0f}); // 0
    p.push_back({{size, startY, 0.0f}, {size, startY, 0.0f}, {0.0f,0.0f,0.0f}, 1.0f}); // 1
    p.push_back({{size, startY, size}, {size, startY, size}, {0.0f,0.0f,0.0f}, 1.0f}); // 2
    p.push_back({{0.0f, startY, size}, {0.0f, startY, size}, {0.0f,0.0f,0.0f}, 1.0f}); // 3
    
    // Top 4 vertices
    p.push_back({{0.0f, startY+size, 0.0f}, {0.0f, startY+size, 0.0f}, {0.0f,0.0f,0.0f}, 1.0f}); // 4
    p.push_back({{size, startY+size, 0.0f}, {size, startY+size, 0.0f}, {0.0f,0.0f,0.0f}, 1.0f}); // 5
    p.push_back({{size, startY+size, size}, {size, startY+size, size}, {0.0f,0.0f,0.0f}, 1.0f}); // 6
    p.push_back({{0.0f, startY+size, size}, {0.0f, startY+size, size}, {0.0f,0.0f,0.0f}, 1.0f}); // 7

    float k = 400.0f; // Stiffness

    // Build the structural springs (same as before)
    CreateSpring(springs, p[0], p[1], k); CreateSpring(springs, p[1], p[2], k);
    CreateSpring(springs, p[2], p[3], k); CreateSpring(springs, p[3], p[0], k);
    CreateSpring(springs, p[4], p[5], k); CreateSpring(springs, p[5], p[6], k);
    CreateSpring(springs, p[6], p[7], k); CreateSpring(springs, p[7], p[4], k);
    CreateSpring(springs, p[0], p[4], k); CreateSpring(springs, p[1], p[5], k);
    CreateSpring(springs, p[2], p[6], k); CreateSpring(springs, p[3], p[7], k);
    CreateSpring(springs, p[0], p[6], k); CreateSpring(springs, p[1], p[7], k);
    CreateSpring(springs, p[2], p[4], k); CreateSpring(springs, p[3], p[5], k);

    // Create an Obstacle (A massive concrete pillar)
    Obstacle pillar = { {-5.0f, 0.0f, -5.0f}, {-2.0f, 10.0f, -2.0f}, GRAY };

    float gravity = -15.0f; 
    float driveSpeed = 50.0f;

    // --- MAIN GAME LOOP ---
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
            
            // Only apply driving force to bottom particles to simulate wheels pushing
            if (particle.position.y < startY + 1.0f) {
                particle.acceleration.x += driveForce.x;
                particle.acceleration.z += driveForce.z;
            }
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

            // Obstacle Collision
            HandleObstacleCollision(particle, pillar);
        }

        // Make the camera follow the car
        camera.target = p[5].position;
        camera.position = { p[5].position.x + 10.0f, 10.0f, p[5].position.z + 15.0f };

        // --- RENDER ---
        BeginDrawing();
            ClearBackground(SKYBLUE);
            
            BeginMode3D(camera);
                DrawGrid(40, 1.0f);
                
                // Draw the Obstacle
                DrawCube(
                    {(pillar.minBounds.x + pillar.maxBounds.x)/2, 
                     (pillar.minBounds.y + pillar.maxBounds.y)/2, 
                     (pillar.minBounds.z + pillar.maxBounds.z)/2}, 
                    pillar.maxBounds.x - pillar.minBounds.x, 
                    pillar.maxBounds.y - pillar.minBounds.y, 
                    pillar.maxBounds.z - pillar.minBounds.z, 
                    pillar.color
                );
                DrawCubeWires(
                    {(pillar.minBounds.x + pillar.maxBounds.x)/2, 
                     (pillar.minBounds.y + pillar.maxBounds.y)/2, 
                     (pillar.minBounds.z + pillar.maxBounds.z)/2}, 
                    pillar.maxBounds.x - pillar.minBounds.x, 
                    pillar.maxBounds.y - pillar.minBounds.y, 
                    pillar.maxBounds.z - pillar.minBounds.z, 
                    DARKGRAY
                );

                // Draw the Polygons (Faces of the cube)
                Color carColor = RED;
                DrawSoftFace(p[0], p[1], p[5], p[4], carColor); // Front
                DrawSoftFace(p[1], p[2], p[6], p[5], carColor); // Right
                DrawSoftFace(p[2], p[3], p[7], p[6], carColor); // Back
                DrawSoftFace(p[3], p[0], p[4], p[7], carColor); // Left
                DrawSoftFace(p[4], p[5], p[6], p[7], carColor); // Top
                DrawSoftFace(p[3], p[2], p[1], p[0], carColor); // Bottom

                // Draw Wireframe over the polygons so you can still see the topology morph
                for (auto& s : springs) {
                    DrawLine3D(s.p1->position, s.p2->position, BLACK);
                }

            EndMode3D();

            DrawText("WASD to Drive | Smash into the pillar!", 10, 10, 20, DARKGRAY);
            DrawFPS(10, 40);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}