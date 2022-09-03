defmodule ToyRobot do
  # max x-coordinate of table top
  @table_top_x 5
  # max y-coordinate of table top
  @table_top_y :e
  # mapping of y-coordinates
  @robot_map_y_atom_to_num %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5}

  @doc """
  Places the robot to the default position of (1, A, North)

  Examples:

      iex> ToyRobot.place
      {:ok, %ToyRobot.Position{facing: :north, x: 1, y: :a}}
  """
  def place do
    {:ok, %ToyRobot.Position{}}
  end

  def place(x, y, _facing) when x < 1 or y < :a or x > @table_top_x or y > @table_top_y do
    {:failure, "Invalid position"}
  end

  def place(_x, _y, facing)
  when facing not in [:north, :east, :south, :west]
  do
    {:failure, "Invalid facing direction"}
  end

  @doc """
  Places the robot to the provided position of (x, y, facing),
  but prevents it to be placed outside of the table and facing invalid direction.

  Examples:

      iex> ToyRobot.place(1, :b, :south)
      {:ok, %ToyRobot.Position{facing: :south, x: 1, y: :b}}

      iex> ToyRobot.place(-1, :f, :north)
      {:failure, "Invalid position"}

      iex> ToyRobot.place(3, :c, :north_east)
      {:failure, "Invalid facing direction"}
  """
  def place(x, y, facing) do
    {:ok, %ToyRobot.Position{x: x, y: y, facing: facing}}
  end

  @doc """
  Provide START position to the robot as given location of (x, y, facing) and place it.
  """
  def start(x, y, facing) do
    ###########################
    ## complete this funcion ##
    ###########################

    {:ok, %ToyRobot.Position{x: x, y: y, facing: facing}}

  end

  def stop(_robot, goal_x, goal_y, _cli_proc_name) when goal_x < 1 or goal_y < :a or goal_x > @table_top_x or goal_y > @table_top_y do
    {:failure, "Invalid STOP position"}
  end

  @doc """
  Provide STOP position to the robot as given location of (x, y) and plan the path from START to STOP.
  Passing the CLI Server process name that will be used to send robot's current status after each action is taken.
  Spawn a process and register it with name ':client_toyrobot' which is used by CLI Server to send an
  indication for the presence of obstacle ahead of robot's current position and facing.
  """
  def stop(robot, goal_x, goal_y, cli_proc_name) do
    ###########################
    ## complete this funcion ##
    ###########################

    pid = spawn_link(fn -> listen(robot, goal_x, goal_y, cli_proc_name) end)
    Process.register(pid, :client_toyrobot)

  end

  @doc """
  Send Toy Robot's current status i.e. location (x, y) and facing
  to the CLI Server process after each action is taken.
  Listen to the CLI Server and wait for the message indicating the presence of obstacle.
  The message with the format: '{:obstacle_presence, < true or false >}'.
  """
  def send_robot_status(%ToyRobot.Position{x: x, y: y, facing: facing} = _robot, cli_proc_name) do
    send(cli_proc_name, {:toyrobot_status, x, y, facing})
    # IO.puts("Sent by Toy Robot Client: #{x}, #{y}, #{facing}")
    listen_from_server()
  end

  @doc """
  Listen to the CLI Server and wait for the message indicating the presence of obstacle.
  The message with the format: '{:obstacle_presence, < true or false >}'.
  """
  def listen_from_server() do
    receive do
      {:obstacle_presence, is_obs_ahead} -> is_obs_ahead
    end
  end

  @doc """
  Provides the report of the robot's current position

  Examples:

      iex> {:ok, robot} = ToyRobot.place(2, :b, :west)
      iex> ToyRobot.report(robot)
      {2, :b, :west}
  """
  def report(%ToyRobot.Position{x: x, y: y, facing: facing} = _robot) do
    {x, y, facing}
  end

  @directions_to_the_right %{north: :east, east: :south, south: :west, west: :north}
  @doc """
  Rotates the robot to the right
  """
  def right(%ToyRobot.Position{facing: facing} = robot) do
    %ToyRobot.Position{robot | facing: @directions_to_the_right[facing]}
  end

  @directions_to_the_left Enum.map(@directions_to_the_right, fn {from, to} -> {to, from} end)
  @doc """
  Rotates the robot to the left
  """
  def left(%ToyRobot.Position{facing: facing} = robot) do
    %ToyRobot.Position{robot | facing: @directions_to_the_left[facing]}
  end

  @doc """
  Moves the robot to the north, but prevents it to fall
  """
  def move(%ToyRobot.Position{x: _, y: y, facing: :north} = robot) when y < @table_top_y do
    %ToyRobot.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the east, but prevents it to fall
  """
  def move(%ToyRobot.Position{x: x, y: _, facing: :east} = robot) when x < @table_top_x do
    %ToyRobot.Position{robot | x: x + 1}
  end

  @doc """
  Moves the robot to the south, but prevents it to fall
  """
  def move(%ToyRobot.Position{x: _, y: y, facing: :south} = robot) when y > :a do
    %ToyRobot.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the west, but prevents it to fall
  """
  def move(%ToyRobot.Position{x: x, y: _, facing: :west} = robot) when x > 1 do
    %ToyRobot.Position{robot | x: x - 1}
  end

  @doc """
  Does not change the position of the robot.
  This function used as fallback if the robot cannot move outside the table
  """
  def move(robot), do: robot

  def failure do
    raise "Connection has been lost"
  end

  '''
  Team defined functions
  '''
  def listen(robot, goal_x, goal_y, cli_proc_name) do

    {x, y, _facing} = report(robot)
    robot = if(x != goal_x or y != goal_y) do
      robot = path_plan_y(robot, goal_x, goal_y, cli_proc_name)
      robot = path_plan_x(robot, goal_x, goal_y, cli_proc_name)
      robot
    else
      send_robot_status(robot, cli_proc_name)
      robot
    end

    {x, y, _facing} = report(robot)
    robot = if(x != goal_x or y != goal_y) do
      robot = listen(robot, goal_x, goal_y, cli_proc_name)
      robot
    end
    {:ok, robot}
  end
  def path_plan_y(robot, goal_x, goal_y, cli_proc_name) do
    {_x, y, facing} = report(robot)
    flag = :y

    robot = if(y != goal_y) do
      y_diff = abs(@robot_map_y_atom_to_num[y] - @robot_map_y_atom_to_num[goal_y])


      robot = if(y < goal_y) do
        cond do
          facing == :south ->
            send_robot_status(robot, cli_proc_name)
            robot = right(robot)
            send_robot_status(robot, cli_proc_name)
            robot = right(robot)
            move_multiple(robot, y_diff, goal_x, goal_y, cli_proc_name, flag)
          facing == :west ->
            send_robot_status(robot, cli_proc_name)
            robot = right(robot)
            move_multiple(robot, y_diff, goal_x, goal_y, cli_proc_name, flag)
          facing == :east ->
            send_robot_status(robot, cli_proc_name)
            robot = left(robot)
            move_multiple(robot, y_diff, goal_x, goal_y, cli_proc_name, flag)
          facing == :north ->
            move_multiple(robot, y_diff, goal_x, goal_y, cli_proc_name, flag)
          true ->
            move_multiple(robot, y_diff, goal_x, goal_y, cli_proc_name, flag)
        end

      else
        cond do
          (y > goal_y and facing == :north) ->
            send_robot_status(robot, cli_proc_name)
            robot = right(robot)
            send_robot_status(robot, cli_proc_name)
            robot = right(robot)
            move_multiple(robot, y_diff, goal_x, goal_y, cli_proc_name, flag)
          (y > goal_y and facing == :east) ->
            send_robot_status(robot, cli_proc_name)
            robot = right(robot)
            move_multiple(robot, y_diff, goal_x, goal_y, cli_proc_name, flag)
          (y > goal_y and facing == :west) ->
            send_robot_status(robot, cli_proc_name)
            robot = left(robot)
            move_multiple(robot, y_diff, goal_x, goal_y, cli_proc_name, flag)
          (y > goal_y and facing == :south) ->
            move_multiple(robot, y_diff, goal_x, goal_y, cli_proc_name, flag)
          true ->
            move_multiple(robot, y_diff, goal_x, goal_y, cli_proc_name, flag)
        end
      end
      robot
    else
      #send_robot_status(robot, cli_proc_name)
      robot
    end
    robot
  end

  def path_plan_x(robot, goal_x, goal_y, cli_proc_name) do
    {x, _y, facing} = report(robot)
    flag = :x

    robot = if(x != goal_x) do
      x_diff = abs(x - goal_x)

      robot = if(x < goal_x) do
        cond do
          facing == :south ->
            send_robot_status(robot, cli_proc_name)
            robot = left(robot)
            move_multiple(robot, x_diff, goal_x, goal_y, cli_proc_name, flag)
          facing == :west ->
            send_robot_status(robot, cli_proc_name)
            robot = right(robot)
            send_robot_status(robot, cli_proc_name)
            robot = right(robot)
            move_multiple(robot, x_diff, goal_x, goal_y, cli_proc_name, flag)
          facing == :east ->
            move_multiple(robot, x_diff, goal_x, goal_y, cli_proc_name, flag)
          facing == :north ->
            send_robot_status(robot, cli_proc_name)
            robot = right(robot)
            move_multiple(robot, x_diff, goal_x, goal_y, cli_proc_name, flag)
          true ->
            move_multiple(robot, x_diff, goal_x, goal_y, cli_proc_name, flag)
        end
      else
        cond do
          (x > goal_x and facing == :north) ->
            send_robot_status(robot, cli_proc_name)
            robot = left(robot)
            move_multiple(robot, x_diff, goal_x, goal_y, cli_proc_name, flag)
          (x > goal_x and facing == :east) ->
            send_robot_status(robot, cli_proc_name)
            robot = right(robot)
            send_robot_status(robot, cli_proc_name)
            robot = right(robot)
            move_multiple(robot, x_diff, goal_x, goal_y, cli_proc_name, flag)
          (x > goal_x and facing == :west) ->
            move_multiple(robot, x_diff, goal_x, goal_y, cli_proc_name, flag)
          (x > goal_x and facing == :south) ->
            send_robot_status(robot, cli_proc_name)
            robot = right(robot)
            move_multiple(robot, x_diff, goal_x, goal_y, cli_proc_name, flag)
          true ->
            move_multiple(robot, x_diff, goal_x, goal_y, cli_proc_name, flag)
        end
      end
      robot
    else
      #send_robot_status(robot, cli_proc_name)
      robot
    end
    robot
  end



  def avoid_obs_y(robot, cli_proc_name) do

    {x, _y, facing} = report(robot)
    robot = if(facing == :north) do
      robot = if(x == 5) do
        robot = left(robot)
        t = send_robot_status(robot, cli_proc_name)
        robot = if(t == false) do
          robot = move(robot)
          send_robot_status(robot, cli_proc_name)
          robot = right(robot)
          #send_robot_status(robot, cli_proc_name)
          robot
        else
          avoid_obs_y(robot, cli_proc_name)
        end
        robot
      else
        robot = right(robot)
        t = send_robot_status(robot, cli_proc_name)
        robot = if(t == false) do
          robot = move(robot)
          send_robot_status(robot, cli_proc_name)
          robot = left(robot)
          #send_robot_status(robot, cli_proc_name)
          robot
        else
          avoid_obs_y(robot, cli_proc_name)
        end
        robot
      end
      robot

    else
      robot = if(x == 5) do
        robot = right(robot)
        t = send_robot_status(robot, cli_proc_name)
        robot = if(t == false) do
          robot = move(robot)
          send_robot_status(robot, cli_proc_name)
          robot = left(robot)
          #send_robot_status(robot, cli_proc_name)
          robot
        else
          avoid_obs_y(robot, cli_proc_name)
        end
        robot
      else
        robot = left(robot)
        t = send_robot_status(robot, cli_proc_name)
        robot = if(t == false) do
          robot = move(robot)
          send_robot_status(robot, cli_proc_name)
          robot = right(robot)
          #send_robot_status(robot, cli_proc_name)
          robot
        else
          avoid_obs_y(robot, cli_proc_name)
        end
        robot
      end
      robot
    end
    robot
  end

  def avoid_obs_x(robot, cli_proc_name) do
    {_x, y, facing} = report(robot)
    robot = if(facing == :east) do
      robot = if(y == :a) do
        robot = left(robot)
        t = send_robot_status(robot, cli_proc_name)
        robot = if(t == false) do
          robot = move(robot)
          send_robot_status(robot, cli_proc_name)
          robot = right(robot)
          #send_robot_status(robot, cli_proc_name)
          robot
        else
          avoid_obs_x(robot, cli_proc_name)
        end
        robot
      else
        robot = right(robot)
        t = send_robot_status(robot, cli_proc_name)
        robot = if(t == false) do
          robot = move(robot)
          send_robot_status(robot, cli_proc_name)
          robot = left(robot)
          #send_robot_status(robot, cli_proc_name)
          robot
        else
          avoid_obs_x(robot, cli_proc_name)
        end
        robot
      end
      robot
    else
      robot = if(y == :a) do
        robot = right(robot)
        t = send_robot_status(robot, cli_proc_name)
        robot = if(t == false) do
          robot = move(robot)
          send_robot_status(robot, cli_proc_name)
          robot = left(robot)
          #send_robot_status(robot, cli_proc_name)
          robot
        else
          avoid_obs_x(robot, cli_proc_name)
        end
        robot
      else
        robot = left(robot)
        t = send_robot_status(robot, cli_proc_name)
        robot = if(t == false) do
          robot = move(robot)
          send_robot_status(robot, cli_proc_name)
          robot = right(robot)
          #send_robot_status(robot, cli_proc_name)
          robot
        else
          avoid_obs_x(robot, cli_proc_name)
        end
        robot
      end
      robot
    end
    robot
  end

  def move_multiple(robot, steps, goal_x, goal_y, cli_proc_name, _flag) when steps <= 0 do

    {x, y, _facing} = report(robot)
    if(x == goal_x and y == goal_y) do
      send_robot_status(robot, cli_proc_name)
    end
    robot
  end

  def move_multiple(robot, steps, goal_x, goal_y, cli_proc_name, flag) when steps > 0 do
    t = send_robot_status(robot, cli_proc_name)
    robot = if(t == true) do
      if(flag == :y) do
        robot = avoid_obs_y(robot, cli_proc_name)
        move_multiple(robot, steps, goal_x, goal_y, cli_proc_name, flag)
      else
        robot = avoid_obs_x(robot, cli_proc_name)
        move_multiple(robot, steps, goal_x, goal_y, cli_proc_name, flag)
      end
    else
      robot = move(robot)
      #send_robot_status(robot, cli_proc_name)
      move_multiple(robot, steps - 1, goal_x, goal_y, cli_proc_name, flag)
    end
    robot
  end
end
