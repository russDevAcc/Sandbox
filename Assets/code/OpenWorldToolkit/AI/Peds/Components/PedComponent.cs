using Unity.Entities;
using Unity.Mathematics;

namespace OpenWorldToolkit.AI.Peds.Components
{
    /// <summary>
    /// Component used for general pedestrian data. The route they are on, their current index in the route. and their current destination!
    /// </summary>
    ///
    public struct PedComponent : IComponentData
    {
        
        public Entity path;
        public int nodeIndex;
        public float3 destination;
        public byte direction;
    }
}