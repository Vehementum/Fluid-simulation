using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;

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
    [Range(0.1f, 5f)] public float particleSize = 0.1f;
    public Vector2 boundsSize = new Vector2(10f, 8f);
    public GameObject particlePrefab; // Assign in Inspector
    public Vector2[] positions;
    public Vector2[] velocities;
    public Vector2[] PredictedPositions; // Store predicted positions for rendering
    private Transform[] particleTransforms; // Store transforms for rendering

    public float[] densities;

    public float Dampening_factor = 0.8f; // Damping factor for velocity

    public float gravity = 9.81f; // Gravity constant
    public float smoothingRadius = 1f; // Smoothing radius for density calculation
    public float targetDensity = 1f; // Target density for pressure calculation
    public float pressureCoefficient = 1f; // Coefficient for pressure calculation

    Dictionary<Vector2Int, List<int>> spatialGrid;
    float cellSize;

    bool isPaused;

    Vector3 mousePos=Vector3.zero;
    bool isPullInteraction;
    bool isPushInteraction;

    public float interactionInputStrength;
    public float rayoninterraction;

    
    
    float currInteractStrength = 0;
    

    
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
        PredictedPositions = new Vector2[particleCount];

        // Place particles in a grid
        for (int i = 0; i < particleCount; i++)
        {
            Vector2 randomPosition = CreateRandomVector(i);
            
            // Initialize the Particle struct
            particles[i] = new Particle { position = randomPosition, velocity = Vector2.zero };
            
            positions[i] = randomPosition;
            PredictedPositions[i] = positions[i];
            velocities[i] = particles[i].velocity;
            densities[i] = 1f; // Initialize density to zero

            // Instantiate the prefab and get its ParticleComponent
            GameObject particle = Instantiate(particlePrefab, new Vector3(randomPosition.x, randomPosition.y, 0f), Quaternion.identity);
            particle.transform.localScale = new Vector3(particleSize, particleSize, particleSize);
            ParticleComponent particleComponent = particle.GetComponent<ParticleComponent>();
            

            // Assign the Particle struct to the component
            particleComponent.particleData = particles[i];

            // Store the particle's transform
            particleTransforms[i] = particle.transform;
        }
        // Initialize spatial grid for optimization
        cellSize = smoothingRadius; // Each cell covers the radius of influence
        spatialGrid = new Dictionary<Vector2Int, List<int>>();
    }

    void Update()
    {
        // Basic_update(); // Call the basic update function
        if(!isPaused){
            SimulationStep(Time.deltaTime); // Call the simulation step function
        }
        HandleInput();
        
        
    }


    void Basic_update()
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
        if (Mathf.Abs(position.x) > halfBoundSize.x - particleSize/10)
        {
            position.x = (halfBoundSize.x - particleSize/10) * Mathf.Sign(position.x);
            velocity.x *= -1f * Dampening_factor; // Reverse velocity on collision
        }

        // Check for Y bound collision
        if (Mathf.Abs(position.y) > halfBoundSize.y - particleSize/10)
        {
            position.y = (halfBoundSize.y - particleSize/10) * Mathf.Sign(position.y);
            velocity.y *= -1f * Dampening_factor; // Reverse velocity on collision
        }
    }
    static float smoothing_kernel(float radius, float dst)
    {
        if (dst > radius) return 0f; // Outside influence radius
        float volume = (8*Mathf.PI * Mathf.Pow(radius,3)) / 3; // Volume of the sphere
        return 4*(radius - dst) * (radius - dst) / volume;
    }

    // static float smoothing_kernel2(float h, float r)
    // {
    //     if (r > h) return 0f;
    //     float a = 4f / (Mathf.PI * Mathf.Pow(h, 8));
    //     return a * Mathf.Pow(h * h - r * r, 3);
    // }
    // static float smoothing_kernel_derivative2(float h, float r)
    // {
    //     if (r > h || r == 0f) return 0f;
    //     float a = -24f / (Mathf.PI * Mathf.Pow(h, 8));
    //     return a * r * Mathf.Pow(h * h - r * r, 2);
    // }

    static float smoothing_kernel_derivative(float radius, float dst)
    {
        if (dst > radius) return 0f; // Outside influence radius
        float scale = 12 / Mathf.Pow(radius, 3) * Mathf.PI; // Scale factor for derivative
        return 4*(dst - radius) * scale;
    }
    float CalculateDensity(Vector2 samplePoint)
    {
       float density = 0f;
       const float mass = 1;

       // Iterate through all particles to calculate density at the sample point
        foreach (int i in GetNeighbors(samplePoint))
        {
            Vector2 position = positions[i];
            float distance = (position - samplePoint).magnitude;
            float influence = smoothing_kernel(smoothingRadius, distance);
            density += mass * influence;
        }
        return density;
    }

    float CalculatePredictedDensity(Vector2 samplePoint)
    {
       float density = 0f;
       const float mass = 1f;

       // Iterate through all particles to calculate density at the sample point
        foreach (int i in GetNeighbors(samplePoint))
        {
            Vector2 position = PredictedPositions[i];
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

    Vector2 CalculatePressureForce(Vector2 samplePoint, int index)
    {
        Vector2 pressureforce = Vector2.zero;
        // Iterate through all particles to calculate property gradient at the sample point
        foreach (int i in GetNeighbors(samplePoint))
        {
            Particle particle = particles[i];
            float mass = 1f; // Mass of the particle (can be adjusted based on your simulation)
            float distance = (positions[i] - samplePoint).magnitude;
            if (distance == 0 || densities[i] == 0)
                continue;
            Vector2 direction = (positions[i] - samplePoint)/distance; // Normalize direction vector
            float slope = smoothing_kernel_derivative(smoothingRadius, distance);
            float density = densities[i];
            float sharedPressure = CalculateSharedPressure(density, densities[index]); // Calculate shared pressure
            pressureforce += - sharedPressure * direction * slope * mass / density;// Pressure force calculation
        }
        return pressureforce;

    }
    Vector2 CalculatePredictPressureForce(Vector2 samplePoint, int index)
    {
        Vector2 pressureforce = Vector2.zero;
        // Iterate through all particles to calculate property gradient at the sample point
        foreach (int i in GetNeighbors(samplePoint))
        {
            Particle particle = particles[i];
            float mass = 1f; // Mass of the particle (can be adjusted based on your simulation)
            float distance = (PredictedPositions[i] - samplePoint).magnitude;
            if (distance == 0 || densities[i] == 0)
                continue;
            Vector2 direction = (PredictedPositions[i] - samplePoint)/distance; // Normalize direction vector
            float slope = smoothing_kernel_derivative(smoothingRadius, distance);
            float density = densities[i];
            float sharedPressure = CalculateSharedPressure(density, densities[index]); // Calculate shared pressure
            pressureforce += - sharedPressure * direction * slope * mass / density;// Pressure force calculation
        }
        return pressureforce;

    }

    float ConvertDensityToPressure(float density)
    {
        float densityerror = density - targetDensity;
        float pressure = densityerror * pressureCoefficient;
        return pressure;
    }

    float ConvertDensityToPressure2(float density) // Prevent explosion near borders
{
    float gamma = 7f; // Tait equation gamma
    float restDensity = targetDensity;
    float stiffness = pressureCoefficient;

    if (density <= 0f) return 0f;
    return stiffness * (Mathf.Pow(density / restDensity, gamma) - 1f);
}

    Vector2 CalculateViscosityForce(Vector2 samplePoint, int index)
    {
        Vector2 viscosityForce = Vector2.zero;
        float mu = 0.1f; // Viscosity coefficient
        float mass = 1f;

        foreach (int i in GetNeighbors(samplePoint))
        {
            if (i == index) continue;
            float distance = (PredictedPositions[i] - samplePoint).magnitude;
            if (distance > smoothingRadius) continue;

            Vector2 velDiff = velocities[i] - velocities[index];
            float influence = smoothing_kernel(smoothingRadius, distance);
            viscosityForce += mu * velDiff * influence / densities[i];
        }

        return viscosityForce;
    }

    void SimulationStep(float deltaTime)
    {
        for(int i = 0; i < particleCount; i++)
        {
            velocities[i] +=Vector2.down * gravity * deltaTime; // Apply gravity to velocity
             // Predict new position
            
            isPullInteraction = Input.GetMouseButton(0);
            isPushInteraction = Input.GetMouseButton(1);
            if (isPushInteraction || isPullInteraction)
                {
                    Vector3 mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
                    
                    currInteractStrength = isPushInteraction ? -interactionInputStrength : interactionInputStrength;
                    velocities[i]+=InteractionForce(mousePos, rayoninterraction, currInteractStrength, i);
                    
                    
                    
                }

            PredictedPositions[i] = positions[i] + velocities[i] * 1/120f;
            ResolveCollisions(ref PredictedPositions[i], ref velocities[i]);
        }
        
        UpdateSpatialGrid();
        
        for(int i = 0; i < particleCount; i++)
        {
            densities[i] = CalculatePredictedDensity(PredictedPositions[i]); // Update density for each particle
        }
        for(int i = 0; i < particleCount; i++){
            Vector2 pressureForce = CalculatePredictPressureForce(PredictedPositions[i], i); // Calculate pressure force
            Vector2 viscosityForce = CalculateViscosityForce(PredictedPositions[i], i); // Calculate viscosity force
            Vector2 pressureAcceleration = pressureForce / densities[i]; // Calculate acceleration from pressure force
            velocities[i] += (pressureAcceleration + viscosityForce) * deltaTime; // Update velocity based on pressure acceleration
        }


        for(int i = 0; i < particleCount; i++){
            positions[i] += velocities[i] * deltaTime; // Update position based on velocity
            ResolveCollisions(ref positions[i], ref velocities[i]); // Resolve collisions with bounds
        }
        for(int i = 0; i < particleCount; i++)
        {
            particles[i].position = positions[i];
            particles[i].velocity = velocities[i];
        // Update the visual prefab position based on the Particle struct data
            particleTransforms[i].position = new Vector3(particles[i].position.x, particles[i].position.y, 0f);

            // Update the ParticleComponent on the particle prefab
            ParticleComponent particleComponent = particleTransforms[i].GetComponent<ParticleComponent>();
            particleComponent.particleData = particles[i];
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

    void UpdateSpatialGrid()
    {
        if (positions == null || positions.Length != particleCount)
            return;
        spatialGrid.Clear();

        for (int i = 0; i < particleCount; i++)
        {
            Vector2Int cell = GetCell(PredictedPositions[i]);
            if (!spatialGrid.ContainsKey(cell))
                spatialGrid[cell] = new List<int>();

            spatialGrid[cell].Add(i);
        }
    }

    Vector2Int GetCell(Vector2 pos)
    {
        return new Vector2Int(
            Mathf.FloorToInt(pos.x / cellSize),
            Mathf.FloorToInt(pos.y / cellSize)
        );
    }

    IEnumerable<int> GetNeighbors(Vector2 position)
    {
        Vector2Int baseCell = GetCell(position);

        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                Vector2Int neighborCell = new Vector2Int(baseCell.x + dx, baseCell.y + dy);
                if (spatialGrid.TryGetValue(neighborCell, out List<int> indices))
                {
                    foreach (int i in indices)
                    {
                        yield return i;
                    }
                }
            }
        }
    }

    float CalculateSharedPressure(float denityA, float densityB)
    {
        float pressureA = ConvertDensityToPressure2(denityA);
        float pressureB = ConvertDensityToPressure2(densityB);
        return (pressureA + pressureB) / 2f;
    }

    Vector2 InteractionForce(Vector2 input, float radius, float strenght, int particleIndex){
        Vector2 InteractionForce = Vector2.zero;
        Vector2 offset = input - positions[particleIndex];

        float sqrDst = offset.x*offset.x + offset.y*offset.y;
        float dst = Mathf.Sqrt(sqrDst);
        
        if(dst < radius){
            
            Vector2 dirtoinputpoint = dst <= 0.01f ? Vector2.zero : offset / dst;
            
            float centreT = 1 - dst / radius;
            InteractionForce += (dirtoinputpoint * strenght -velocities[particleIndex])* centreT;
        }
        return InteractionForce;
    }
    
    
    
    
    
    void HandleInput()
		{
			if (Input.GetKeyDown(KeyCode.Space))
			{
				isPaused = !isPaused;
			}
            if (Input.GetMouseButton(0)){
               
            }
        }

    // void OnDrawGizmos()
    // {
    //     if (!isPaused)
    //     {
    //         Vector2 mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
    //         bool isPullInteraction = Input.GetMouseButton(0);
    //         bool isPushInteraction = Input.GetMouseButton(1);
    //         bool isInteracting = isPullInteraction || isPushInteraction;
    //         if (isInteracting)
    //         {
    //             Gizmos.color = isPullInteraction ? Color.green : Color.red;
    //             Gizmos.DrawWireSphere(mousePos, 1);
    //         }
    //     }
    // }
    
}
