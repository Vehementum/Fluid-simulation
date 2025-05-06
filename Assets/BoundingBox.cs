using UnityEngine;
using System.Linq;
[RequireComponent(typeof(LineRenderer))]
public class BoundsRenderer : MonoBehaviour
{
    public Vector2 boundsSize = new Vector2(10.5f, 8.5f);

    void Start()
    {
        LineRenderer lr = GetComponent<LineRenderer>();
        lr.positionCount = 5; // square + back to start
        lr.loop = false;
        lr.useWorldSpace = true;
        lr.widthMultiplier = 0.05f;

        
        
        
    }
    public void UpdateBox()
{
    LineRenderer lr = GetComponent<LineRenderer>();
    lr.positionCount = 5;
    lr.loop = false;
    lr.widthMultiplier = 0.05f;
    lr.useWorldSpace = true;

    if (lr.material == null || lr.material.shader.name != "Sprites/Default")
{
    lr.material = new Material(Shader.Find("Sprites/Default"));
    lr.startColor = Color.white;
    lr.endColor = Color.white;
}

    float w = boundsSize.x / 2f;
    float h = boundsSize.y / 2f;

    Vector3[] corners = new Vector3[]
    {
        new Vector3(-w, -h, 0f),
        new Vector3(-w,  h, 0f),
        new Vector3( w,  h, 0f),
        new Vector3( w, -h, 0f),
        new Vector3(-w, -h, 0f)
    };

    lr.SetPositions(corners);
    // Debug.Log($"Bounds Size: {boundsSize}, Corners: {string.Join(", ", corners.Select(c => c.ToString()))}");
}
}
