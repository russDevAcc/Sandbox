using OpenWorldToolkit.AI.Peds.Components;
using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.AI;

namespace OpenWorldToolkit.AI.Peds
{

    public class PedMoveSystem : SystemBase
    {
        public static int GetNextIndexFromDirection(int currentIndex, byte direction, ref DynamicBuffer<Node> buffer)
        {
            int next = currentIndex;
            if (direction == 0)
                next = (currentIndex >= buffer.Length - 1) ? 0 : currentIndex + 1;
            else
                next = (currentIndex == 0) ? buffer.Length - 1 : currentIndex - 1;
            return next;
        }

        public static Vector3 GetPerpendicularVector(Vector3 from, Vector3 to, float offset)
        {
            Vector3 newVector = (to - from);
            newVector = Vector3.Normalize(Vector3.Cross(newVector, new Vector3(0, -1, 0))) * offset + from;
            return newVector;
        }

        /// <summary>
        /// Called when a ped reaches its current destination and updates it with a new position to go to.
        /// </summary>
        /// <param name="ped"></param>
        /// <param name="coord"></param>
        /// <param name="agent"></param>
        private void ProceedToNextNode(ref PedComponent ped, ref Coord coord, NavMeshAgent agent)
        {
            if (EntityManager.Exists(ped.path) == false)
                return;

            //Gets the buffer of nodes that each path contains.
            DynamicBuffer<Node> buffer = GetBuffer<Node>(ped.path);

            //Gets the next index from the direction assigned to the ped at spawn
            int next = GetNextIndexFromDirection(ped.nodeIndex, ped.direction, ref buffer);

            //Updates entities position and moves it to follow the NavMeshAgents GameObject.
            //See PedSpawnSystem for more info on the companions.
            coord.position = agent.transform.position;

            //Create a capture variable since PedComponent is a by value type.
            PedComponent p = ped;
            //Sets the nodes current index to the next index available in the path. 
            p.nodeIndex = next;

            //caches the next relevant node
            next = GetNextIndexFromDirection(ped.nodeIndex, ped.direction, ref buffer);

            //Create a new destination, a random value between the offsets of the next node. This
            //ensures the pedestrians stay spread out rather than follow a line.
            //and makes the system appear more natural.




            var dest = buffer[next].position;
           // Debug.Log(dest.ToString());
            //Sets the NavMeshAgent's destination 
            agent.SetDestination(dest);

            //updates the components destination with the agents destination.
            
            p.destination = agent.destination;

            //assign the value of the ped component to our capture variable.
            ped = p;
        }
        protected override void OnUpdate()
        {
            Entities.WithNone<NeedsCompanion>().ForEach((Entity entity, ref Coord coord, ref PedComponent ped) =>
            {
                //We aren't interested in any peds that don't have a NavMeshAgent attached yet.
                if (EntityManager.HasComponent<NavMeshAgent>(entity) == true)
                {

                    NavMeshAgent agent = EntityManager.GetComponentObject<NavMeshAgent>(entity);
                    if (math.distance(agent.transform.position, ped.destination) <= 2.1f)
                    {
                        ProceedToNextNode(ref ped, ref coord, agent);
                    }
                }
            }).WithoutBurst().Run();
        }
    }
}