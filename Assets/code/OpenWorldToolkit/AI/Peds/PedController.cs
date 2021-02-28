using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.AI;
namespace OpenWorldToolkit.AI.Peds
{
    public class PedPool
    {
        private readonly Stack<NavMeshAgent> peds;

        public PedPool(int maxPeds, List<Ped> pedModels)
        {
            peds = new Stack<NavMeshAgent>(maxPeds);

            foreach (var ped in pedModels)
            {
                AddPedToPool(ped);
            }

            if (pedModels.Count < maxPeds)
            {
                while (peds.Count < maxPeds)
                {
                    if (pedModels == null)
                        continue;
                    if (pedModels.Count == 0)
                        return;
                    AddPedToPool(pedModels[UnityEngine.Random.Range(0, pedModels.Count - 1)]);
                }
            }
        }

        private void AddPedToPool(Ped ped)
        {
            var go = GameObject.Instantiate(ped.model);
            var animator = go.GetComponent<Animator>();
            if (animator == null)
                animator = go.AddComponent<Animator>();
            animator.runtimeAnimatorController = ped.animatorController;
            go.SetActive(false);
            go.transform.position = new float3(-1000, -1000, -1000);
            var agent = go.GetComponentInChildren<NavMeshAgent>();
            
            peds.Push(agent);

        }

        public NavMeshAgent Rent()
        {

            return peds.Pop();

        }

        public void Return(NavMeshAgent agent)
        {
            agent.transform.position = new float3(-1000, -1000, -1000);
            agent.gameObject.SetActive(false);
            peds.Push(agent);
        }

        public int count
        {
            get
            {
                return peds.Count;
            }
        }
    }


    public class PedController : MonoBehaviour
    {
        public int maxPeds;

        public List<PathInstance> paths;


        public List<Ped> peds;

        [HideInInspector]
        public PedPool pedPool;

        private void Start()
        {
            pedPool = new PedPool(maxPeds, peds);
        }

    }
}