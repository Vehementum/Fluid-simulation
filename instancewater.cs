using UnityEditor.Experimental.GraphView;
using UnityEngine;

public class instancewater : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        for (int i = 0; i < number; i++)
        {
            Instantiate(water, new Vector3(Random.Range(-5f,5f),0,0), transform.rotation);
        }
        
    }
    public GameObject water;
    public int number;
    
    void Update()
    {
        
    }
}
