using UnityEngine;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine.Timeline;
using UnityEngine.UIElements;


public class waterscript : MonoBehaviour
{
    Vector2 boundsSize;
    void Start()
    {
        boundsSize.x = 11;
        boundsSize.y = 4;
    }
    public Rigidbody2D body;
    public float gravity;
    Vector2 position;
    Vector2 velocity;
    
    public float particleSize;

    void Update()
    {
        velocity += Vector2.down*gravity*Time.deltaTime;
        position += velocity * Time.deltaTime;
        ResolveColisions();
        body.position = position;
    }

    void ResolveColisions()
    {
        Vector2 halfboundsize = boundsSize / 2 - Vector2.one * particleSize;

        if (Mathf.Abs(position.x) > halfboundsize.x)
        {
            position.x = halfboundsize.x * Mathf.Sign(position.x);
            velocity.x *= -1;
        }

        if (Mathf.Abs(position.y) > halfboundsize.y)
        {
            position.y = halfboundsize.y * Mathf.Sign(position.y);
            velocity.y *= -1;
        }
    }

    
}
