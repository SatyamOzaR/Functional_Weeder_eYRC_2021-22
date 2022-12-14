defmodule ToyRobot.PhoenixSocketClient do

  alias PhoenixClient.{Socket, Channel, Message}

  @doc """
  Connect to the Phoenix Server URL (defined in config.exs) via socket.
  Once ensured that socket is connected, join the channel on the server with topic "robot:status".
  Get the channel's PID in return after joining it.

  NOTE:
  The socket will automatically attempt to connect when it starts.
  If the socket becomes disconnected, it will attempt to reconnect automatically.
  Please note that start_link is not synchronous,
  so you must wait for the socket to become connected before attempting to join a channel.
  Reference to above note: https://github.com/mobileoverlord/phoenix_client#usage

  You may refer: https://github.com/mobileoverlord/phoenix_client/issues/29#issuecomment-660518498
  """
  def connect_server do
    ###########################
    ## complete this funcion ##
    ###########################

    #--------------------------------------------------------------#
    # socket URL from {config.exs}
    url_from_config = Application.get_env(:phoenix_server, :url)
    socket_opts = [
      url: url_from_config
    ]

    # connect to socket
    {:ok, socket} = PhoenixClient.Socket.start_link(socket_opts)

    # wait until socket is connected
    wait_until_connected(socket)

    # join channel with topic as "robot:status"
    PhoenixClient.Channel.join(socket, "robot:status")

  end

  # waiting function
  defp wait_until_connected(socket) do

    # check if, socket is connected?
    if !PhoenixClient.Socket.connected?(socket) do

      # sleep
      Process.sleep(100)

      # callback again
      wait_until_connected(socket)

    end

  end

  #-------------------------------------------------------------------#

  @doc """
  Send Toy Robot's current status i.e. location (x, y) and facing
  to the channel's PID with topic "robot:status" on Phoenix Server with the event named "new_msg". The message to be sent should be a Map.
  In return from Phoenix server, receive the boolean value < true OR false > indicating the obstacle's presence
  in this format: {:ok, < true OR false >}.
  Create a tuple of this format: '{:obstacle_presence, < true or false >}' as a return of this function.
  """
  def send_robot_status(channel, %ToyRobot.Position{x: x, y: y, facing: facing} = _robot) do
    ###########################
    ## complete this funcion ##
    ###########################

    #------------------------------------------------------------------#

    # create map with x, y & facing values
    m = %{"x" => x, "y" => to_string(y), "face" => to_string(facing)}

    # send map to channel with event "new_msg"
    {:ok, obs} = PhoenixClient.Channel.push(channel, "new_msg", m)

    # get obstacle presence <True or False>
    {:obstacle_presence, obs}

    #------------------------------------------------------------------#

  end

end
