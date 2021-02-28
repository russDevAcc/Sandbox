using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace OpenWorldToolkit.AI.Peds
{
    [CreateAssetMenu]
    public class Ped : ScriptableObject
    {
        public GameObject model;
        public RuntimeAnimatorController animatorController;
        public AIBehavior[] behaviors;
    }
}