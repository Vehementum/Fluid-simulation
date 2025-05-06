using UnityEngine;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine.Timeline;
using UnityEngine.UIElements;
public struct Particle
{
    public Vector2 position;
    public Vector2 velocity;
    public Vector2 acceleration;

    public Particle(Vector2 startPosition)
    {
        position = startPosition;
        velocity = Vector2.zero;
        acceleration = Vector2.zero;
    }
}