using Newtonsoft.Json;
using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
namespace OpenWorldToolkit.AI
{
    public struct Node : IBufferElementData
    {
        public int index;
        public float width;
        public float3 position;
    }

    public struct Path : IComponentData
    {
        public int numPeds;
    }
}
