using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace OpenWorldToolkit.AI.Peds
{
    [UpdateBefore(typeof(PedSpawnSystem))]
    [UpdateAfter(typeof(PedDespawnSystem))]
    public class PedPathRelevancySystem : SystemBase
    {
        EntityCommandBufferSystem barrier => World.GetOrCreateSystem<EndSimulationEntityCommandBufferSystem>();
        protected override void OnUpdate()
        {
            var pos = Camera.main.transform.position;
            var commandBuffer = barrier.CreateCommandBuffer().AsParallelWriter();
            Entities
                .WithNone<NeedsCompanion>()
                .ForEach((Entity entity, int entityInQueryIndex,
                    int nativeThreadIndex, ref Path path, ref DynamicBuffer<Node> buffer) =>
                {
                    foreach (var node in buffer)
                    {
                        if (math.distance(node.position, pos) <= 300)
                        {
                            commandBuffer.AddComponent<Relevant>(nativeThreadIndex, entity);
                        }
                        else
                        {
                            commandBuffer.RemoveComponent<Relevant>(nativeThreadIndex, entity);
                        }
                    }
                }).ScheduleParallel();
            barrier.AddJobHandleForProducer(Dependency);
        }
    }
}