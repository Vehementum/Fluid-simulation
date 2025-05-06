using UnityEngine;

public class ParticleComponent : MonoBehaviour
{
    // The Particle struct for each particle object
    public FluidSimulator.Particle particleData;

    // Optionally, update the position of the visual object based on particle data
    void Update()
    {
        transform.position = new Vector3(particleData.position.x, particleData.position.y, 0f);
    }
}
