
using Unity.Entities;
/// <summary>
/// Used to tag spawned peds without gameobjects. Added when ped entity is spawned and removed when a gameobject is binded to an entity.
/// </summary>
public struct NeedsCompanion : IComponentData
{

}
