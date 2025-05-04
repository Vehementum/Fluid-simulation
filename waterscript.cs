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
        boundsSize.x = 12;
        boundsSize.y = 8;
        
       
    }
    public float gravity=20;
    Vector2 position;
    Vector2 velocity;

    public float colisionfactor=1;
    
    public float particleSize;

    void Update()
    {
        position = transform.position;
        velocity += Vector2.down*gravity*Time.deltaTime;
        
        position += velocity * Time.deltaTime;
        ResolveColisions();
        transform.position = position;
        
    }

    void ResolveColisions()
    {
        Vector2 halfboundsize = boundsSize / 2;

        if (Mathf.Abs(position.x) > halfboundsize.x)
        {
            position.x = halfboundsize.x * Mathf.Sign(position.x);
            velocity.x *= -1*colisionfactor;
        }

        if (Mathf.Abs(position.y) > halfboundsize.y)
        {
            position.y = halfboundsize.y * Mathf.Sign(position.y);
            velocity.y *= -1*colisionfactor;
        }
    }

    
}
