package edu.umkc.ic
import java.io._
import java.net._
/**
 * Created by Rajashekar176 on 7/29/2015.
 */
object iOSConnector {
  def findIpAdd():String =
  {
    val localhost = InetAddress.getLocalHost
    val localIpAddress = localhost.getHostAddress

    return localIpAddress
  }
  def sendCommandToRobot(string: String)
  {
    // Simple server

    try {


      lazy val address: Array[Byte] = Array(10.toByte, 205.toByte, 1.toByte, 81.toByte)
      val ia = InetAddress.getByAddress(address)
      val socket = new Socket(ia, 5555)
      val out = new PrintStream(socket.getOutputStream)
      //val in = new DataInputStream(socket.getInputStream())

      out.print(string)
      out.flush()

      out.close()
      //in.close()
      socket.close()
    }
    catch {
      case e: IOException =>
        e.printStackTrace()
    }
  }


}