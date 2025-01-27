# ROS
import rclpy
from rclpy.node import Node

# Package
from dm_network_info_msgs.msg import NetworkStatus
from visplot.real_time_plotter import *


class PlotterManager(Node):
    """
    ROS node to manage the creation, initialisation and maintenance of four data plots

    ...
    Attributes
    ----------
    target_rsu_id : str
        The RSU ID the data of which to plot
    target_obu_id : str
        The OBU ID the data of which to plot 
    plot_data_store_delay : visplot.real_time_plotter.PlotDataStore
        The storage for the delay values of the RSU-OBU connection
    plot_data_store_jitter : visplot.real_time_plotter.PlotDataStore
        The storage for the jitter values of the RSU-OBU connection
    plot_data_store_rssi : visplot.real_time_plotter.PlotDataStore
        The storage for the rssi values of the RSU-OBU connection
    plot_data_store_packet_loss : visplot.real_time_plotter.PlotDataStore
        The storage for the packet loss values of the RSU-OBU connection
    real_time_plotter : visplot.real_time_plotter.RealTimePlotter
        The plot manager responsible for updating graphs
    subscription : rclpy.Subscription
        The subscriber to the target RSU-OBU network status topic
    
    Methods
    -------
    update_graphs(message: dm_network_info_msgs.NetworkStatus):
        Updates the graph points based on the incoming network status message 
    """
    def __init__(self):
        """
        Constructor
        """
        super().__init__("plotter_manager")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_rsu_id', 'RSU_0'),
                ('target_obu_id', 'OBU_0')
            ]
        )

        self.target_rsu_id = self.get_parameter('target_rsu_id').value
        self.target_obu_id = self.get_parameter('target_obu_id').value
        
        # The data stores
        self.plot_data_store_delay_object_info = PlotDataStore("ObjectInfo Delay - Time", "Time", "Delay")
        self.plot_data_store_delay_freespace_info = PlotDataStore("FreespaceInfo Delay - Time", "Time", "Delay")
        self.plot_data_store_delay_signal_info = PlotDataStore("SignalInfo Delay - Time", "Time", "Delay")
        self.plot_data_store_jitter_object_info = PlotDataStore("ObjectInfo Jitter - Time", "Time", "Jitter")
        self.plot_data_store_jitter_freespace_info = PlotDataStore("FreespaceInfo Jitter - Time", "Time", "Jitter")
        self.plot_data_store_jitter_signal_info = PlotDataStore("SignalInfo Jitter - Time", "Time", "Jitter")
        self.plot_data_store_rssi_object_info = PlotDataStore("ObjectInfo RSSI - Time", "Time", "RSSI")
        self.plot_data_store_rssi_freespace_info = PlotDataStore("FreespaceInfo RSSI - Time", "Time", "RSSI")
        self.plot_data_store_rssi_signal_info = PlotDataStore("SignalInfo RSSI - Time", "Time", "RSSI")
        self.plot_data_store_packet_loss_object_info = PlotDataStore("ObjectInfo Packet Loss - Time", "Time", "Packet Loss")
        self.plot_data_store_packet_loss_freespace_info = PlotDataStore("FreespaceInfo Packet Loss - Time", "Time", "Packet Loss")
        self.plot_data_store_packet_loss_signal_info = PlotDataStore("SignalInfo Packet Loss - Time", "Time", "Packet Loss")

        # The plot manager
        self.real_time_plotter = RealTimePlotter(
            self.plot_data_store_delay_object_info,
            self.plot_data_store_delay_freespace_info,
            self.plot_data_store_delay_signal_info,
            self.plot_data_store_jitter_object_info,
            self.plot_data_store_jitter_freespace_info,
            self.plot_data_store_jitter_signal_info,
            self.plot_data_store_rssi_object_info,
            self.plot_data_store_rssi_freespace_info,
            self.plot_data_store_rssi_signal_info,
            self.plot_data_store_packet_loss_object_info,
            self.plot_data_store_packet_loss_freespace_info,
            self.plot_data_store_packet_loss_signal_info)

        # The subscriber on the given topic
        self.object_info_subscription = self.create_subscription(
            NetworkStatus,
            "/" + self.target_obu_id + "/" + self.target_rsu_id + "/object_info/network_status",
            self.update_graphs_object_info,
            10)

        # The subscriber on the given topic
        self.freespace_info_subscription = self.create_subscription(
            NetworkStatus,
            "/" + self.target_obu_id + "/" + self.target_rsu_id + "/freespace_info/network_status",
            self.update_graphs_freespace_info,
            10)

        # The subscriber on the given topic
        self.signal_info_subscription = self.create_subscription(
            NetworkStatus,
            "/" + self.target_obu_id + "/" + self.target_rsu_id + "/signal_info/network_status",
            self.update_graphs_signal_info,
            10)


    def update_graphs_object_info(self, message: NetworkStatus) -> None:
        """
        Callback function that updates the graph points for delay, jitter, 
        RSSI and packet loss based on the incoming network status message

        Parameters
        ----------
        message : dm_network_info_msgs.NetworkStatus
            The message containing new delay, jitter, RSSI and packet loss data to 
            append to the graphs
        
        Returns
        -------
        None
        """
        new_delay_point = PointTimeValue(
            message.stamp.sec + message.stamp.nanosec * 1e-9, message.delay)

        new_jitter_point = PointTimeValue(
            message.stamp.sec + message.stamp.nanosec * 1e-9, message.jitter)

        new_rssi_point = PointTimeValue(
            message.stamp.sec + message.stamp.nanosec * 1e-9, message.rssi)
        
        new_packet_loss_point = PointTimeValue(
            message.stamp.sec + message.stamp.nanosec * 1e-9, message.packet_loss)

        self.plot_data_store_delay_object_info.add_new_timevalue_point_directly(new_delay_point)
        self.plot_data_store_jitter_object_info.add_new_timevalue_point_directly(new_jitter_point)
        self.plot_data_store_rssi_object_info.add_new_timevalue_point_directly(new_rssi_point)
        self.plot_data_store_packet_loss_object_info.add_new_timevalue_point_directly(new_packet_loss_point)

        try:
            self.real_time_plotter.update()
        except KeyboardInterrupt:
            pass

    def update_graphs_freespace_info(self, message: NetworkStatus) -> None:
        """
        Callback function that updates the graph points for delay, jitter, 
        RSSI and packet loss based on the incoming network status message

        Parameters
        ----------
        message : dm_network_info_msgs.NetworkStatus
            The message containing new delay, jitter, RSSI and packet loss data to 
            append to the graphs
        
        Returns
        -------
        None
        """
        new_delay_point = PointTimeValue(
            message.stamp.sec + message.stamp.nanosec * 1e-9, message.delay)

        new_jitter_point = PointTimeValue(
            message.stamp.sec + message.stamp.nanosec * 1e-9, message.jitter)

        new_rssi_point = PointTimeValue(
            message.stamp.sec + message.stamp.nanosec * 1e-9, message.rssi)
        
        new_packet_loss_point = PointTimeValue(
            message.stamp.sec + message.stamp.nanosec * 1e-9, message.packet_loss)

        self.plot_data_store_delay_freespace_info.add_new_timevalue_point_directly(new_delay_point)
        self.plot_data_store_jitter_freespace_info.add_new_timevalue_point_directly(new_jitter_point)
        self.plot_data_store_rssi_freespace_info.add_new_timevalue_point_directly(new_rssi_point)
        self.plot_data_store_packet_loss_freespace_info.add_new_timevalue_point_directly(new_packet_loss_point)

        try:
            self.real_time_plotter.update()
        except KeyboardInterrupt:
            pass

    def update_graphs_signal_info(self, message: NetworkStatus) -> None:
        """
        Callback function that updates the graph points for delay, jitter, 
        RSSI and packet loss based on the incoming network status message

        Parameters
        ----------
        message : dm_network_info_msgs.NetworkStatus
            The message containing new delay, jitter, RSSI and packet loss data to 
            append to the graphs
        
        Returns
        -------
        None
        """
        new_delay_point = PointTimeValue(
            message.stamp.sec + message.stamp.nanosec * 1e-9, message.delay)

        new_jitter_point = PointTimeValue(
            message.stamp.sec + message.stamp.nanosec * 1e-9, message.jitter)

        new_rssi_point = PointTimeValue(
            message.stamp.sec + message.stamp.nanosec * 1e-9, message.rssi)
        
        new_packet_loss_point = PointTimeValue(
            message.stamp.sec + message.stamp.nanosec * 1e-9, message.packet_loss)

        self.plot_data_store_delay_signal_info.add_new_timevalue_point_directly(new_delay_point)
        self.plot_data_store_jitter_signal_info.add_new_timevalue_point_directly(new_jitter_point)
        self.plot_data_store_rssi_signal_info.add_new_timevalue_point_directly(new_rssi_point)
        self.plot_data_store_packet_loss_signal_info.add_new_timevalue_point_directly(new_packet_loss_point)

        try:
            self.real_time_plotter.update()
        except KeyboardInterrupt:
            pass


def main(args=None):
    rclpy.init(args=args)

    plotter_manager = PlotterManager()

    rclpy.spin(plotter_manager)

    plt.close()

    plotter_manager.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()