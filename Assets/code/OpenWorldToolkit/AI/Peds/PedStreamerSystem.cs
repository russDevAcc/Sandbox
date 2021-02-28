using Newtonsoft.Json;
using System.IO;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using Unity.Jobs;
using UnityEngine.AI;
using Unity.Jobs.LowLevel.Unsafe;
using OpenWorldToolkit.AI.Peds.Components;

namespace OpenWorldToolkit.AI.Peds
{
    [UpdateBefore(typeof(PedPathRelevancySystem))]
    public class PedDespawnSystem : SystemBase
    {
        PedSpawnSystem streamerSystem;
        PedController pedController;

        protected override void OnCreate()
        {
            streamerSystem = World.GetExistingSystem<PedSpawnSystem>();
            pedController = GameObject.FindObjectOfType<PedController>();
        }

        protected override void OnUpdate()
        {
            var pos = Camera.main.transform.position;
            Entities.WithStructuralChanges().WithNone<NeedsCompanion>().ForEach((
       Entity entity, int entityInQueryIndex, ref PedComponent ped, ref Coord coord) =>
            {
                if (math.distance(coord.position, pos) <= 300)
                    return;

                if (EntityManager.HasComponent<NavMeshAgent>(entity) == false)
                    return;

                var agent = EntityManager.GetComponentObject<NavMeshAgent>(entity);

                var path = EntityManager.GetComponentData<Path>(ped.path);
                path.numPeds -= 1;
                EntityManager.SetComponentData(ped.path, path);
                pedController.pedPool.Return(agent);
                EntityManager.DestroyEntity(entity);
                streamerSystem.numPeds.Value -= 1;

            }).WithoutBurst().Run();
        }
    }
}

