using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

namespace OpenWorldToolkit.AI.Peds.Editor
{
    public class PathCreatorWindow : EditorWindow
    {
        private static EditorWindow window;


        private PedController pedController;

        private PathInstance path;

        private bool creatingPath;

        private bool drawNodes;

        [MenuItem("Tools/Open World Toolkit/Path creator.")]
        public static void ShowWindow()
        {
            window = EditorWindow.GetWindow(typeof(PathCreatorWindow));

        }

        private void OnEnable()
        {
            pedController = FindObjectOfType<PedController>();
            SceneView.duringSceneGui += SceneView_duringSceneGui;
        }

        private void OnGUI()
        {
            drawNodes = GUILayout.Toggle(drawNodes, new GUIContent() { text = "Draw Nodes?" });
            if (creatingPath == false)
                ShowOptions();

            else
                ShowNodeHelp();
        }



        private void ShowOptions()
        {
            if (GUILayout.Button("Create path"))
            {

                var go = new GameObject("path");
                path = go.AddComponent<PathInstance>();
                creatingPath = true;
                pedController.paths.Add(path);
            }
        }

        private string pathName;
        private void ShowNodeHelp()
        {
            pathName = EditorGUILayout.TextField(new GUIContent() { text = "Route Name" }, pathName);
            EditorGUILayout.HelpBox("Use alt + left click to create a node", MessageType.Info);


            if (GUILayout.Button("Finish"))
            {
                creatingPath = false;
                path.gameObject.name = pathName;

                Vector3 averagePosition = Vector3.zero;


                foreach (var node in path.nodes)
                {
                    averagePosition += node.transform.position;
                }

                averagePosition = averagePosition / path.nodes.Count;
                path.transform.position = averagePosition;
                foreach (var node in path.nodes)
                {
                    node.transform.SetParent(path.transform);
                }
                cleanUp();
            }
        }

        private void SceneView_duringSceneGui(SceneView obj)
        {


            if (creatingPath == false)
                return;

            Event e = Event.current;
            if (e.type == EventType.MouseDown && e.button == 0 && e.alt == true)
            {

                Vector3 mousePos = e.mousePosition;
                float ppp = EditorGUIUtility.pixelsPerPoint;
                mousePos.y = obj.camera.pixelHeight - mousePos.y * ppp;
                mousePos.x *= ppp;

                Ray ray = obj.camera.ScreenPointToRay(mousePos);
                RaycastHit hit;

                if (Physics.Raycast(ray, out hit))
                {
                    CreateNode(hit.point);
                }
            }


        }




        private void CreateNode(float3 position)
        {
            var go = new GameObject();
            go.transform.position = position;
            // go.transform.SetParent(path.transform);

            var node = go.AddComponent<NodeInstance>();
            path.nodes.Add(node);

            int index = path.nodes.IndexOf(node);
            node.index = index;
            go.name = "node_" + index;
        }


        private void cleanUp()
        {
            creatingPath = false;

        }

        private void OnDisable()
        {
            pedController = null;

            SceneView.duringSceneGui -= SceneView_duringSceneGui;
            cleanUp();
        }
    }
}
