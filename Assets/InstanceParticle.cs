using UnityEngine;

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
    [Range(10, 10000)] public int particleCount = 1000;
    [Range(0.1f, 5f)] public float particleSpacing = 0.2f;
    [Range(0.1f, 5f)] public float particleSize = 0.5f;
    public Vector2 boundsSize = new Vector2(10f, 10f);
    public GameObject particlePrefab; // Assign in Inspector
    private Transform[] particleTransforms; // Store transforms for rendering

    public float Dampening_factor = 0.8f; // Damping factor for velocity

    public float gravity = 9.81f; // Gravity constant

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

        // Place particles in a grid
        int particlesPerRow = Mathf.CeilToInt(Mathf.Sqrt(particleCount));
        float spacing = particleSize * 2f + particleSpacing;

        for (int i = 0; i < particleCount; i++)
        {
            float x = (i % particlesPerRow - particlesPerRow / 2f + 0.5f) * spacing;
            float y = (i / particlesPerRow - particlesPerRow / 2f + 0.5f) * spacing;
            
            // Initialize the Particle struct
            particles[i] = new Particle { position = new Vector2(x, y), velocity = Vector2.zero };

            // Instantiate the prefab and get its ParticleComponent
            GameObject particle = Instantiate(particlePrefab, new Vector3(x, y, 0f), Quaternion.identity);
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
    }

    void ResolveCollisions(ref Vector2 position, ref Vector2 velocity)
    {
        Vector2 halfBoundSize = boundsSize / 2;

        // Check for X bound collision
        if (Mathf.Abs(position.x) > halfBoundSize.x)
        {
            position.x = halfBoundSize.x * Mathf.Sign(position.x);
            velocity.x *= -1f * Dampening_factor; // Reverse velocity on collision
        }

        // Check for Y bound collision
        if (Mathf.Abs(position.y) > halfBoundSize.y)
        {
            position.y = halfBoundSize.y * Mathf.Sign(position.y);
            velocity.y *= -1f * Dampening_factor; // Reverse velocity on collision
        }
    }
    
}
