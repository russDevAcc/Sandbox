using Unity.Entities;
using Unity.Mathematics;

/// <summary>
/// Used to store an entities position.
/// </summary>
public struct Coord : IComponentData
{
    public float3 position;
}
