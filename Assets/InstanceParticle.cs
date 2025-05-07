using UnityEngine;
using Unity.Mathematics;
public class FluidSimulator : MonoBehaviour
{
    // Particle struct for simulation data
    public struct Particle
    {
        public Vector2 position;
        public Vector2 velocity;
    }

    ////////////////////////////////////////////////// Constants for simulation ///////////////////////////////////////////////////////
    private Particle[] particles; // Array to hold all particles
    [Range(10, 10000)] public int particleCount = 10;
    [Range(0.1f, 5f)] public float particleSpacing = 0.2f;
    [Range(0.1f, 5f)] public float particleSize = 0.5f;
    public Vector2 boundsSize = new Vector2(10f, 8f);
    public GameObject particlePrefab; // Assign in Inspector
    public Vector2[] positions;
    public Vector2[] velocities;
    private Transform[] particleTransforms; // Store transforms for rendering

    public float[] densities;

    public float Dampening_factor = 0.8f; // Damping factor for velocity

    public float gravity = 9.81f; // Gravity constant
    public float smoothingRadius = 1f; // Smoothing radius for density calculation
    public float targetDensity = 1f; // Target density for pressure calculation
    public float pressureCoefficient = 1f; // Coefficient for pressure calculation

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void Start()
    {
        // Set camera position and size
        Camera cam = Camera.main;
        cam.transform.position = new Vector3(0, 0, -10); // looking at origin
        float margin = 1f;
        cam.orthographic = true;
        cam.orthographicSize = boundsSize.y / 2f + margin;

        float screenRatio = (float)Screen.width / Screen.height;
        float targetRatio = boundsSize.x / boundsSize.y;
        // Fit width if too narrow
        if (screenRatio < targetRatio)
        {
            cam.orthographicSize = (boundsSize.x / screenRatio) / 2f + margin;
        }
        // Initialize bounds size
        BoundsRenderer boundsRenderer = FindObjectOfType<BoundsRenderer>();
        if (boundsRenderer != null)
        {
            boundsRenderer.boundsSize = boundsSize;
            boundsRenderer.UpdateBox(); // Call the method that updates the box after setting size
        }

        // Initialize particle arrays
        particles = new Particle[particleCount];
        particleTransforms = new Transform[particleCount];
        positions = new Vector2[particleCount];
        velocities = new Vector2[particleCount];
        densities = new float[particleCount];

        // Place particles in a grid
        for (int i = 0; i < particleCount; i++)
        {
            Vector2 randomPosition = CreateRandomVector(i);
            
            // Initialize the Particle struct
            particles[i] = new Particle { position = randomPosition, velocity = Vector2.one };
            positions[i] = randomPosition;
            velocities[i] = particles[i].velocity;
            densities[i] = 1f; // Initialize density to zero

            // Instantiate the prefab and get its ParticleComponent
            GameObject particle = Instantiate(particlePrefab, new Vector3(randomPosition.x, randomPosition.y, 0f), Quaternion.identity);
            ParticleComponent particleComponent = particle.GetComponent<ParticleComponent>();

            // Assign the Particle struct to the component
            particleComponent.particleData = particles[i];

            // Store the particle's transform
            particleTransforms[i] = particle.transform;
        }
    }

    void Update()
    {
        // Update particles' positions and velocities
        for (int i = 0; i < particleCount; i++)
        {
            // Apply gravity to the velocity
            particles[i].velocity += Vector2.down * Time.deltaTime * gravity;

            // Update the particle's position
            particles[i].position += particles[i].velocity * Time.deltaTime;

            // Resolve collisions (bounds checking)
            ResolveCollisions(ref particles[i].position, ref particles[i].velocity);

            // Update the visual prefab position based on the Particle struct data
            particleTransforms[i].position = new Vector3(particles[i].position.x, particles[i].position.y, 0f);

            // Update the ParticleComponent on the particle prefab
            ParticleComponent particleComponent = particleTransforms[i].GetComponent<ParticleComponent>();
            particleComponent.particleData = particles[i];
        }
        // SimulationStep(Time.deltaTime); // Call the simulation step function
    }

    void ResolveCollisions(ref Vector2 position, ref Vector2 velocity)
    {
        Vector2 halfBoundSize = boundsSize / 2;

        // Check for X bound collision
        if (Mathf.Abs(position.x) > halfBoundSize.x - 0.1f)
        {
            position.x = (halfBoundSize.x - 0.1f) * Mathf.Sign(position.x);
            velocity.x *= -1f * Dampening_factor; // Reverse velocity on collision
        }

        // Check for Y bound collision
        if (Mathf.Abs(position.y) > halfBoundSize.y - 0.1f)
        {
            position.y = (halfBoundSize.y - 0.1f) * Mathf.Sign(position.y);
            velocity.y *= -1f * Dampening_factor; // Reverse velocity on collision
        }
    }
    static float smoothing_kernel(float radius, float dst)
    {
        if (dst > radius) return 0f; // Outside influence radius
        float volume = (Mathf.PI * Mathf.Pow(radius,4)) / 6; // Volume of the sphere
        return (radius - dst) * (radius - dst) / volume;
    }
    static float smoothing_kernel_derivative(float radius, float dst)
    {
        if (dst > radius) return 0f; // Outside influence radius
        float scale = 12 / Mathf.Pow(radius, 4) * Mathf.PI; // Scale factor for derivative
        return (dst - radius) * scale;
    }
    float CalculateDensity(Vector2 samplePoint)
    {
       float density = 0f;
       const float mass = 1;

       // Iterate through all particles to calculate density at the sample point
        foreach(Vector2 position in positions)
        {
            float distance = (position - samplePoint).magnitude;
            float influence = smoothing_kernel(smoothingRadius, distance);
            density += mass * influence;
        }
        return density;
    }
    
    float CalculateProperty(Vector2 samplePoint)
    {
        float property = 0f;
        const float mass = 1;

        // Iterate through all particles to calculate property at the sample point
        foreach(Particle particle in particles)
        {
            float distance = (particle.position - samplePoint).magnitude;
            float influence = smoothing_kernel_derivative(smoothingRadius, distance);
            property += mass * influence * particle.velocity.magnitude;
        }
        return property;
    }

    void updateDensities()
    {
        for(int i = 0; i < particleCount; i++)
        {
            densities[i] = CalculateDensity(particles[i].position);
        };
    }

    Vector2 CalculatePressureForce(Vector2 samplePoint)
    {
        Vector2 pressureforce = Vector2.zero;
        // Iterate through all particles to calculate property gradient at the sample point
        for(int i = 0; i < particleCount; i++)
        {
            Particle particle = particles[i];
            float mass = 1f; // Mass of the particle (can be adjusted based on your simulation)
            float distance = (positions[i] - samplePoint).magnitude;
            Vector2 direction = (positions[i] - samplePoint)/distance; // Normalize direction vector
            float slope = smoothing_kernel_derivative(smoothingRadius, distance);
            float density = densities[i];
            pressureforce += - ConvertDensityToPressure(density) * direction * slope * mass / density;// Pressure force calculation
        }
        return pressureforce;

    }

    float ConvertDensityToPressure(float density)
    {
        float densityerror = density - targetDensity;
        float pressure = densityerror * pressureCoefficient;
        return pressure;
    }

    void SimulationStep(float deltaTime)
    {
        for(int i = 0; i < particleCount; i++)
        {
            velocities[i] +=Vector2.down * gravity * deltaTime; // Apply gravity to velocity
            densities[i] = CalculateDensity(positions[i]); // Update density for each particle
        }
        for(int i = 0; i < particleCount; i++){
            Vector2 pressureForce = CalculatePressureForce(positions[i]); // Calculate pressure force
            Vector2 pressureAcceleration = pressureForce / densities[i]; // Calculate acceleration from pressure force
            velocities[i] += pressureAcceleration * deltaTime; // Update velocity based on pressure acceleration
        }
        for(int i = 0; i < particleCount; i++){
            positions[i] += velocities[i] * deltaTime; // Update position based on velocity
            ResolveCollisions(ref positions[i], ref velocities[i]); // Resolve collisions with bounds
        }
    }

    Vector2 CreateRandomVector(int index)
    {
        float x = (float)(UnityEngine.Random.Range(0f, 1f) -0.5) * boundsSize.x;
        float y = (float)(UnityEngine.Random.Range(0f, 1f) -0.5) * boundsSize.y;
        return new Vector2(x, y);
    }

    Vector2 CreateGridVector(int index)
    {
        float x = (index % Mathf.Sqrt(particleCount)) * particleSpacing - boundsSize.x / 2f;
        float y = (index / Mathf.Sqrt(particleCount)) * particleSpacing - boundsSize.y / 2f;
        return new Vector2(x, y);
    }
}
