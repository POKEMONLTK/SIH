using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace PGKSimulationLauncher
{
    class Program
    {
        private static TcpListener _tcpServer;
        private static string _baseDir;
        private static int _port = 8080;
        private static bool _running = true;
        private static string _localIp = "127.0.0.1";

        static void Main(string[] args)
        {
            Console.Title = "155mm PGK Digital Twin & GNC Simulation (PC & Android)";
            Console.OutputEncoding = Encoding.UTF8;

            _baseDir = AppDomain.CurrentDomain.BaseDirectory;
            Directory.SetCurrentDirectory(_baseDir);

            PrintBanner();

            // Ensure standalone single-file HTML is present on disk for Android/offline copying
            string standalonePath = Path.Combine(_baseDir, "PGK_Simulation_Standalone.html");
            if (!File.Exists(standalonePath))
            {
                byte[] embedded = GetEmbeddedSimulationHtml();
                if (embedded != null && embedded.Length > 0)
                {
                    try { File.WriteAllBytes(standalonePath, embedded); } catch { }
                }
            }

            _localIp = GetBestLocalIP();
            StartTcpHttpServer();

            string localUrl = "http://localhost:" + _port + "/";
            string mobileUrl = "http://" + _localIp + ":" + _port + "/";

            Console.ForegroundColor = ConsoleColor.Cyan;
            Console.WriteLine("  [PC / LAPTOP URL]: " + localUrl);
            Console.ForegroundColor = ConsoleColor.Yellow;
            Console.WriteLine("  [ANDROID PHONE / TABLET URL]: " + mobileUrl);
            Console.ResetColor();
            Console.WriteLine("\n[*] Launching system web browser on PC...");
            OpenUrl(localUrl);

            Console.WriteLine();
            Console.ForegroundColor = ConsoleColor.Green;
            Console.WriteLine("===============================================================================");
            Console.WriteLine("  SIMULATION STATUS: ACTIVE & RUNNING (PC & ANDROID READY)");
            Console.WriteLine("===============================================================================");
            Console.ResetColor();
            Console.WriteLine("  📱 Connect Android: Connect phone to same Wi-Fi and open " + mobileUrl);
            Console.WriteLine("     (Chrome will prompt: 'Install App' or 'Add to Home Screen')");
            Console.WriteLine("-------------------------------------------------------------------------------");
            Console.WriteLine("  [1] Re-open Simulation in PC Browser");
            Console.WriteLine("  [2] Run 6-DOF Monte Carlo Benchmark (Python)");
            Console.WriteLine("  [3] Run TinyML Sensor Health Integration (Python)");
            Console.WriteLine("  [4] Open Project Folder in Windows Explorer");
            Console.WriteLine("  [Q] Stop Simulation & Exit");
            Console.WriteLine("-------------------------------------------------------------------------------");

            while (_running)
            {
                Console.Write("\nEnter choice [1-4, Q]: ");
                string line = null;
                try
                {
                    line = Console.ReadLine();
                }
                catch
                {
                    // Stdin redirected or non-interactive: stay alive until process termination
                    Thread.Sleep(Timeout.Infinite);
                    break;
                }

                if (line == null)
                {
                    Thread.Sleep(1000);
                    continue;
                }

                line = line.Trim().ToUpperInvariant();
                if (line == "Q") break;

                switch (line)
                {
                    case "1":
                        Console.WriteLine("[*] Opening " + localUrl + " in PC browser...");
                        OpenUrl(localUrl);
                        break;
                    case "2":
                        RunPythonScript("run_benchmark.py");
                        break;
                    case "3":
                        RunPythonScript("ai_health_integration.py");
                        break;
                    case "4":
                        Process.Start("explorer.exe", _baseDir);
                        break;
                    default:
                        Console.WriteLine("Invalid option. Enter 1, 2, 3, 4, or Q.");
                        break;
                }
            }

            StopServer();
            Console.WriteLine("\n[+] Server stopped cleanly. Goodbye!");
            Thread.Sleep(800);
        }

        private static void PrintBanner()
        {
            Console.ForegroundColor = ConsoleColor.Green;
            Console.WriteLine(@"
╔═════════════════════════════════════════════════════════════════════════════╗
║       155MM PRECISION GUIDANCE KIT (PGK) & SMART FUZE DIGITAL TWIN          ║
║               SIH 2026 | Problem Statement ID: SIH26098                     ║
║              Yantra India Limited - Ministry of Defence                     ║
╚═════════════════════════════════════════════════════════════════════════════╝");
            Console.ResetColor();
            Console.WriteLine("  Web-Based 3D Digital Twin + 6-DOF Ballistic RK4 + 8-MEMS TinyML Diagnostics\n");
        }

        private static string GetBestLocalIP()
        {
            try
            {
                var host = Dns.GetHostEntry(Dns.GetHostName());
                foreach (var ip in host.AddressList)
                {
                    if (ip.AddressFamily == AddressFamily.InterNetwork)
                    {
                        string s = ip.ToString();
                        // Prefer standard private LAN IP addresses (192.168.x.x, 10.x.x.x, 172.16-31.x.x)
                        if (s.StartsWith("192.168.") && !s.StartsWith("192.168.56.")) return s; // Ignore standard virtualbox
                        if (s.StartsWith("10.")) return s;
                        if (s.StartsWith("172.")) return s;
                    }
                }
                foreach (var ip in host.AddressList)
                {
                    if (ip.AddressFamily == AddressFamily.InterNetwork && !ip.ToString().StartsWith("127."))
                        return ip.ToString();
                }
            }
            catch { }
            return "127.0.0.1";
        }

        private static void StartTcpHttpServer()
        {
            int[] ports = new int[] { 8080, 8000, 8081, 8888, 5000 };

            foreach (int p in ports)
            {
                try
                {
                    _tcpServer = new TcpListener(IPAddress.Any, p);
                    _tcpServer.Start();
                    _port = p;

                    Thread t = new Thread(ListenTcpLoop);
                    t.IsBackground = true;
                    t.Start();

                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.WriteLine("[+] Embedded Universal Web Server listening on 0.0.0.0:" + _port);
                    Console.ResetColor();
                    return;
                }
                catch (Exception)
                {
                    _tcpServer = null;
                }
            }

            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine("[X] Could not start embedded TCP server. Checking fallback...");
            Console.ResetColor();
        }

        private static void ListenTcpLoop()
        {
            while (_running && _tcpServer != null)
            {
                try
                {
                    TcpClient client = _tcpServer.AcceptTcpClient();
                    ThreadPool.QueueUserWorkItem(HandleTcpClient, client);
                }
                catch
                {
                    break;
                }
            }
        }

        private static void HandleTcpClient(object obj)
        {
            using (TcpClient client = (TcpClient)obj)
            {
                try
                {
                    client.ReceiveTimeout = 3000;
                    client.SendTimeout = 5000;
                    using (NetworkStream stream = client.GetStream())
                    {
                        byte[] buffer = new byte[4096];
                        int bytesRead = stream.Read(buffer, 0, buffer.Length);
                        if (bytesRead <= 0) return;

                        string request = Encoding.ASCII.GetString(buffer, 0, bytesRead);
                        string[] lines = request.Split(new string[] { "\r\n", "\n" }, StringSplitOptions.None);
                        if (lines.Length == 0) return;

                        string[] reqParts = lines[0].Split(' ');
                        if (reqParts.Length < 2 || reqParts[0] != "GET")
                        {
                            SendResponse(stream, 400, "Bad Request", "text/plain", Encoding.UTF8.GetBytes("Bad Request"));
                            return;
                        }

                        string rawPath = reqParts[1].Split('?')[0].TrimStart('/');
                        if (string.IsNullOrEmpty(rawPath) || rawPath == "/" || rawPath == "index.html" || rawPath == "PGK_Simulation_Standalone.html")
                        {
                            byte[] embedded = GetEmbeddedSimulationHtml();
                            if (embedded != null && embedded.Length > 0)
                            {
                                SendResponse(stream, 200, "OK", "text/html; charset=utf-8", embedded);
                                return;
                            }
                            rawPath = "index.html";
                        }

                        rawPath = rawPath.Replace('/', Path.DirectorySeparatorChar);
                        string filePath = Path.Combine(_baseDir, rawPath);

                        if (File.Exists(filePath))
                        {
                            byte[] fileBytes = File.ReadAllBytes(filePath);
                            string mime = GetContentType(filePath);
                            SendResponse(stream, 200, "OK", mime, fileBytes);
                        }
                        else
                        {
                            // If index.html or standalone requested and not on disk, serve embedded
                            byte[] embedded = GetEmbeddedSimulationHtml();
                            if (embedded != null && embedded.Length > 0 && (rawPath.EndsWith(".html") || rawPath == "index.html"))
                            {
                                SendResponse(stream, 200, "OK", "text/html; charset=utf-8", embedded);
                                return;
                            }
                            byte[] notFound = Encoding.UTF8.GetBytes("404 Not Found: " + rawPath);
                            SendResponse(stream, 404, "Not Found", "text/plain", notFound);
                        }
                    }
                }
                catch { }
            }
        }

        private static byte[] _cachedEmbeddedHtml = null;
        private static byte[] GetEmbeddedSimulationHtml()
        {
            if (_cachedEmbeddedHtml != null) return _cachedEmbeddedHtml;
            try
            {
                var assembly = System.Reflection.Assembly.GetExecutingAssembly();
                using (Stream s = assembly.GetManifestResourceStream("PGK_Simulation_Standalone.html"))
                {
                    if (s != null)
                    {
                        using (MemoryStream ms = new MemoryStream())
                        {
                            s.CopyTo(ms);
                            _cachedEmbeddedHtml = ms.ToArray();
                            return _cachedEmbeddedHtml;
                        }
                    }
                }
            }
            catch { }
            return null;
        }

        private static void SendResponse(NetworkStream stream, int code, string status, string mime, byte[] body)
        {
            StringBuilder sb = new StringBuilder();
            sb.Append("HTTP/1.1 ").Append(code).Append(" ").Append(status).Append("\r\n");
            sb.Append("Content-Type: ").Append(mime).Append("\r\n");
            sb.Append("Content-Length: ").Append(body.Length).Append("\r\n");
            sb.Append("Access-Control-Allow-Origin: *\r\n");
            sb.Append("Cache-Control: no-cache\r\n");
            sb.Append("Connection: close\r\n\r\n");

            byte[] headerBytes = Encoding.ASCII.GetBytes(sb.ToString());
            stream.Write(headerBytes, 0, headerBytes.Length);
            stream.Write(body, 0, body.Length);
            stream.Flush();
        }

        private static string GetContentType(string path)
        {
            string ext = Path.GetExtension(path).ToLowerInvariant();
            switch (ext)
            {
                case ".html": case ".htm": return "text/html; charset=utf-8";
                case ".js": case ".mjs": return "application/javascript; charset=utf-8";
                case ".css": return "text/css; charset=utf-8";
                case ".json": return "application/json; charset=utf-8";
                case ".png": return "image/png";
                case ".jpg": case ".jpeg": return "image/jpeg";
                case ".svg": return "image/svg+xml";
                case ".pdf": return "application/pdf";
                case ".ico": return "image/x-icon";
                default: return "application/octet-stream";
            }
        }

        private static void OpenUrl(string url)
        {
            try
            {
                Process.Start(new ProcessStartInfo(url) { UseShellExecute = true });
            }
            catch
            {
                try { Process.Start("cmd.exe", "/c start " + url); } catch { }
            }
        }

        private static void RunPythonScript(string scriptName)
        {
            string fullPath = Path.Combine(_baseDir, scriptName);
            if (!File.Exists(fullPath))
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine("[X] File not found: " + scriptName);
                Console.ResetColor();
                return;
            }

            Console.ForegroundColor = ConsoleColor.Yellow;
            Console.WriteLine("\n[+] Executing Python Script: " + scriptName + " ...");
            Console.ResetColor();

            try
            {
                ProcessStartInfo psi = new ProcessStartInfo("python", "\"" + fullPath + "\"")
                {
                    WorkingDirectory = _baseDir,
                    UseShellExecute = false
                };
                using (Process proc = Process.Start(psi))
                {
                    proc.WaitForExit();
                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.WriteLine("\n[+] Process finished with exit code " + proc.ExitCode);
                    Console.ResetColor();
                }
            }
            catch (Exception ex)
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine("[X] Failed to run python script: " + ex.Message);
                Console.ResetColor();
            }
        }

        private static void StopServer()
        {
            if (_tcpServer != null)
            {
                try { _tcpServer.Stop(); } catch { }
                _tcpServer = null;
            }
        }
    }
}
