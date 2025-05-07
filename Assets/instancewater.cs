using System.Collections.Generic;
using System;
using System.Linq;
using NUnit.Framework;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

public class instancewater : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        for (int i = 0; i < number; i++)
        {

            instance=Instantiate(water, new Vector3(UnityEngine.Random.Range(-5f, 5f), 0, 0), transform.rotation);
            
            waterList.Add(instance);
        }
        
    }
    public GameObject water;
    public int number;
    public GameObject instance;
    public List<GameObject> waterList = new List<GameObject> {};


    void Update()
    {

    }
}
