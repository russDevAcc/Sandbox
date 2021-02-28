using Newtonsoft.Json;
using OpenWorldToolkit.AI.Peds.Components;
using OpenWorldToolkit.Common;
using System.IO;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.AI;

namespace OpenWorldToolkit.AI.Peds
{
    /// <summary>
    /// Loops through all relevant ped paths and spawns peds at a 
    /// </summary>
    public class PedSpawnSystem : SystemBase
    {

        private PedController pedController;
        public NativeReference<int> numPeds;
        EntityCommandBufferSystem barrier => World.GetOrCreateSystem<EndSimulationEntityCommandBufferSystem>();


        protected override void OnCreate()
        {
            numPeds = new NativeReference<int>(Allocator.Persistent);
            pedController = GameObject.FindObjectOfType<PedController>();
            LoadPaths();
        }

        protected override void OnDestroy()
        {
            numPeds.Dispose();
        }

        protected override unsafe void OnUpdate()
        {
            var randomArray = World.GetExistingSystem<RandomSystem>().RandomArray;
            var pos = Camera.main.transform.position;

            var peds = numPeds.Value;
            var maxPeds = pedController.maxPeds;
            var commandBuffer = barrier.CreateCommandBuffer().AsParallelWriter();

            var maxPedsPerPath = maxPeds / numPaths;

            Entities
                .WithNativeDisableParallelForRestriction(randomArray)
                .WithNone<NeedsCompanion>().WithAll<Relevant>()
                .ForEach(
           (Entity entity, int entityInQueryIndex, int nativeThreadIndex, ref Path path, ref DynamicBuffer<Node> buffer) =>
            {

                if (peds <= maxPeds && path.numPeds <= maxPedsPerPath)
                {
                    var random = randomArray[nativeThreadIndex];
                    int idx = random.NextInt(0, buffer.Length);
                    randomArray[nativeThreadIndex] = random;



                    var node = buffer[idx];
                    if (peds >= maxPeds)
                        return;

                    byte dir = (byte)random.NextInt(0, 2);
                    int next = PedMoveSystem.GetNextIndexFromDirection(node.index, dir, ref buffer);

                    //var random = randomArray[nativeThreadIndex];

                    var spawn = math.lerp(node.position, buffer[next].position, random.NextFloat(0f, 2f));


                    randomArray[nativeThreadIndex] = random;

                    var e = commandBuffer.CreateEntity(entityInQueryIndex);
                    commandBuffer.AddComponent(entityInQueryIndex, e, new Coord() { position = spawn });
                    commandBuffer.AddComponent(entityInQueryIndex, e, new PedComponent() { nodeIndex = idx, path = entity, destination = spawn, direction = dir });
                    commandBuffer.AddComponent(entityInQueryIndex, e, new NeedsCompanion());
                    path.numPeds += 1;


                }
            }).WithBurst().ScheduleParallel();
            barrier.AddJobHandleForProducer(Dependency);

            Entities.WithStructuralChanges().WithAll<NeedsCompanion>().WithReadOnly(numPeds).ForEach((Entity entity, int entityInQueryIndex, ref PedComponent ped, ref Coord coord) =>
            {


                if (pedController.pedPool == null)
                    return;
                if (numPeds.Value <= maxPeds)
                {
                    if (pedController.pedPool.count <= 0)
                        return;
                    NavMeshAgent agent = pedController.pedPool.Rent();
                    agent.avoidancePriority = UnityEngine.Random.Range(0, 100);
                    agent.obstacleAvoidanceType = ObstacleAvoidanceType.LowQualityObstacleAvoidance;
                    
                    agent.transform.position = coord.position;
                    agent.speed = UnityEngine.Random.Range(1, 4);
                //agent.SetDestination(coord.position);
                numPeds.Value++;
                    agent.gameObject.SetActive(true);


                    EntityManager.RemoveComponent<NeedsCompanion>(entity);
                    EntityManager.AddComponentObject(entity, agent);

                }
            }).WithoutBurst().Run();
        }

        int numPaths = 0;


        private void LoadPaths()
        {
            //

            foreach (var path in pedController.paths)
            {
                if (path == null)
                    continue;
                var pe = EntityManager.CreateEntity();
                EntityManager.AddComponentData<Path>(pe, new Path());

                var buffer = EntityManager.AddBuffer<Node>(pe);
                foreach (var node in path.nodes)
                {
                    if (node == null)
                        continue;
                    int idx = buffer.Add(new Node());
                    var nodeInstance = new Node();
                    nodeInstance.position = new float3(node.transform.position.x, node.transform.position.y, node.transform.position.z);
                    nodeInstance.index = path.nodes.IndexOf(node);
                    buffer[idx] = nodeInstance;
                }
                numPaths += 1;
            }




        }
    }
}