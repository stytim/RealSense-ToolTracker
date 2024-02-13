using UnityEngine;
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using System.Runtime.InteropServices;



#if !UNITY_EDITOR && UNITY_METRO
using Windows.Networking.Sockets;
using Windows.Networking.Connectivity;
using Windows.Networking;
using Windows.Storage.Streams;
#else
using System.Net;
using System.Net.Sockets;
using System.Threading;
#endif
public class UDPToolTrackingReceiver : MonoBehaviour
{

    public int port = 12345; // The port number should match the one you are sending on.
    public int toolID = 1; // The tool ID should match the one you are sending on.
    private Queue<ToolTrackingData> receivedUDPPacketQueue = new Queue<ToolTrackingData>();

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi, Pack = 1)]
    public struct ToolTrackingData
    {
        public long serialNumber; 
        public double timestamp;
        public float posX;
        public float posY;
        public float posZ;
        public float rotX;
        public float rotY;
        public float rotZ;
        public float rotW;
        public int toolId;
    }

    public ToolTrackingData GetLatestUDPPacket()
    {
        ToolTrackingData trackingData = new ToolTrackingData();
        trackingData.toolId = -1;
        while (receivedUDPPacketQueue.Count > 0)
        {
            trackingData = receivedUDPPacketQueue.Dequeue();
        }
        return trackingData;
    }

    private string objectName;

    private void Awake()
    {
        objectName = name;
    }

#if !UNITY_EDITOR && UNITY_METRO

    DatagramSocket socket;

    async void Start()
    {
        socket = new DatagramSocket();
        socket.MessageReceived += Socket_MessageReceived;
        try
        {
            await socket.BindServiceNameAsync(port.ToString());
        }
        catch (Exception e)
        {
            Debug.Log(e.ToString());
            Debug.Log(SocketError.GetStatus(e.HResult).ToString());
            return;
        }
        Debug.Log(objectName + ": DatagramSocket setup done...");
    }

    private void Socket_MessageReceived(DatagramSocket sender, DatagramSocketMessageReceivedEventArgs args)
    {
        try
        {
            // The using statement ensures that the reader is disposed after usage
            using (DataReader reader = args.GetDataReader())
            {
                byte[] receivedBytes = new byte[reader.UnconsumedBufferLength];
                reader.ReadBytes(receivedBytes);

                // Convert the bytes received into the structure we're expecting
                ToolTrackingData trackingData = ByteArrayToStructure<ToolTrackingData>(receivedBytes);
                // Debug.Log($"Received: Timestamp - {trackingData.timestamp}, Position - ({trackingData.posX}, {trackingData.posY}, {trackingData.posZ}), Rotation - ({trackingData.rotX}, {trackingData.rotY}, {trackingData.rotZ}, {trackingData.rotW}), Tool ID - {trackingData.toolId}");
                // Filter by toolID if necessary
                if (trackingData.toolId == toolID)
                {
                    receivedUDPPacketQueue.Enqueue(trackingData);
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Socket_MessageReceived Exception: {e}");
        }
    }

    private void OnDestroy()
    {
        if (socket != null)
        {
            socket.MessageReceived -= Socket_MessageReceived;
            socket.Dispose();
            Debug.Log(objectName + ": Socket disposed");
        }
    }

#else

    private Thread receiveThread;
    private UdpClient client;
    private bool isReceiving = true;
    void Start()
    {
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
        Debug.Log("UDPToolTrackingReceiver: Started on port " + port);
    }

    private void ReceiveData()
    {
        client = new UdpClient(port);
        IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
        while (isReceiving)
        {
            try
            {
                byte[] data = client.Receive(ref anyIP);
                ToolTrackingData trackingData = ByteArrayToStructure<ToolTrackingData>(data);
                // Debug.Log($"Received: Timestamp - {trackingData.timestamp}, Position - ({trackingData.posX}, {trackingData.posY}, {trackingData.posZ}), Rotation - ({trackingData.rotX}, {trackingData.rotY}, {trackingData.rotZ}, {trackingData.rotW}), Tool ID - {trackingData.toolId}");
                if (trackingData.toolId == toolID)
                {
                    receivedUDPPacketQueue.Enqueue(trackingData);
                }
            }
            catch (ObjectDisposedException)
            {

            }
            catch (Exception e)
            {
                Debug.LogError("UDPToolTrackingReceiver: Received Exception " + e.Message);
            }
        }
    }

    void OnDestroy()
    {
        isReceiving = false;
        receiveThread.Abort();
        client.Close();
        Debug.Log("UDPToolTrackingReceiver: Stopped receiving UDP on port " + port);
    }
#endif
    // Helper method to convert byte array to a structure
    T ByteArrayToStructure<T>(byte[] bytes) where T : struct
    {
        GCHandle handle = GCHandle.Alloc(bytes, GCHandleType.Pinned);
        T stuff = (T)Marshal.PtrToStructure(handle.AddrOfPinnedObject(), typeof(T));
        handle.Free();
        return stuff;
    }


    private void Update()
    {
        ToolTrackingData trackingData = GetLatestUDPPacket();
        if (trackingData.toolId != -1)
        {
            // Right-handed to left-handed coordinate system conversion
            transform.position = new Vector3(trackingData.posX, trackingData.posZ, trackingData.posY);
            transform.rotation = new Quaternion(trackingData.rotX, trackingData.rotZ, trackingData.rotY, - trackingData.rotW);
        }
    }


}