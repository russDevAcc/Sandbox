using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;

namespace OpenWorldToolkit.AI.Peds
{
    [ExecuteInEditMode]
#endif
    public class PathInstance : MonoBehaviour
    {
        public List<NodeInstance> nodes = new List<NodeInstance>();

#if UNITY_EDITOR
        [Range(0, 1)]
        public float nodeFudgeFactor = 0.3f;
        [Range(0, 1)]
        public float insertFudgeFactor = 0.175f;
        private GUIStyle indexStyle;
        private void OnEnable()
        {
            indexStyle = new GUIStyle();
            indexStyle.normal.textColor = Color.red;
            indexStyle.fontSize = 20;

            SceneView.duringSceneGui += OnSceneGUI;
        }
        private void OnDisable()
        {
            SceneView.duringSceneGui -= OnSceneGUI;
        }


        void OnSceneGUI(UnityEditor.SceneView view)
        {
            foreach (var node in nodes)
            {
                Vector3 pos = Vector3.Lerp(node.transform.position, nodes[GetNext(node)].transform.position, 0.5f);


                Vector3 diff = node.transform.position - nodes[GetNext(node)].transform.position;



                float nodeSize = HandleUtility.GetHandleSize(node.transform.position) * nodeFudgeFactor;
                nodeSize = Mathf.Clamp(nodeSize, -1, 1);

                if (Handles.Button(node.transform.position, Quaternion.identity, nodeSize, nodeSize, Handles.SphereHandleCap))
                {
                    Selection.activeGameObject = node.gameObject;
                }

                Handles.zTest = UnityEngine.Rendering.CompareFunction.LessEqual;
                Handles.DrawLine(node.transform.position, nodes[GetNext(node)].transform.position);
                Handles.zTest = UnityEngine.Rendering.CompareFunction.Always;

                Handles.Label(node.transform.position, node.index.ToString(), indexStyle);


                float insertSize = HandleUtility.GetHandleSize(pos) * insertFudgeFactor;
                insertSize = Mathf.Clamp(insertSize, -0.5f, 0.5f);

                if (Handles.Button(pos, Quaternion.identity, insertSize, insertSize, Handles.DotHandleCap))
                {
                    var go = new GameObject();
                    go.transform.position = pos;
                    // go.transform.SetParent(path.transform);

                    var n = go.AddComponent<NodeInstance>();

                    nodes.Insert(GetNext(node), n);
                    go.transform.SetParent(transform);

                    foreach (var nodee in nodes)
                    {
                        nodee.index = nodes.IndexOf(nodee);
                        nodee.gameObject.name = "node_" + nodee.index;
                    }
                    break;
                }

            }

        }

        private int GetNext(NodeInstance node)
        {
            return (node.index >= nodes.Count - 1) ? 0 : node.index + 1;
        }
#endif
    }
}