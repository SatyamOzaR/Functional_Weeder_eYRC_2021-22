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
    #--------------------------#
    place(x, y, facing)
    #--------------------------#
  end

  def stop(_robot, goal_x, goal_y, _channel) when goal_x < 1 or goal_y < :a or goal_x > @table_top_x or goal_y > @table_top_y do
    {:failure, "Invalid STOP position"}
  end

  @doc """
  Provide STOP position to the robot as given location of (x, y) and plan the path from START to STOP.
  Passing the channel PID on the Phoenix Server that will be used to send robot's current status after each action is taken.
  Make a call to ToyRobot.PhoenixSocketClient.send_robot_status/2
  to get the indication of obstacle presence ahead of the robot.
  """
  def stop(robot, goal_x, goal_y, channel) do

    ###########################
    ## complete this funcion ##
    ###########################
    flag = %{}
    is_obs_ahead = send_status(channel, robot)
    {:ok, robot} = path_plan_x_y(robot, goal_x, goal_y, is_obs_ahead,channel,flag)
    {:ok, robot}

  end

  def send_status(channel, robot) do
    {:obstacle_presence, obs} = ToyRobot.PhoenixSocketClient.send_robot_status(channel, robot)
    obs
  end

  def path_plan_x_y(robot, goal_x, goal_y, is_obs_ahead, channel, flag) do

    {x,y,_facing} = report(robot)

    flag = cond do
      ((is_obs_ahead) and (not Map.has_key?(flag, robot))) -> Map.put(flag, robot, 100)
      true -> flag
    end

    {:ok, robot} = cond do

      ((x == goal_x) and (y == goal_y)) -> {:ok, robot}

      true ->

          {forward, left, right, flag} = cost_solution(robot, goal_x, goal_y, is_obs_ahead, flag)

          {robot, flag, is_obs_ahead} = cond do

            ((forward <= right) and (forward <= left) and (forward != 100)) ->
              robot = move(robot)
              is_obs_ahead = send_status(channel, robot)
              {robot, flag, is_obs_ahead}

            ((right <= left) and (right != 100)) ->
              robot = right(robot)
              is_obs_ahead = send_status(channel, robot)
              {robot, flag, is_obs_ahead}

            ((right >= left) and (left != 100)) ->
              robot = left(robot)
              is_obs_ahead = send_status(channel, robot)
              {robot, flag, is_obs_ahead}

            true ->
              robot = right(robot)
              _is_obs_ahead = send_status(channel, robot)
              robot = right(robot)
              is_obs_ahead = send_status(channel, robot)
              {robot,flag,is_obs_ahead}

          end
          {:ok, robot} = path_plan_x_y(robot, goal_x, goal_y, is_obs_ahead, channel, flag)
          {:ok, robot}
    end
    {:ok, robot}
  end

  def sol2(%ToyRobot.Position{x: x, y: y, facing: :north} = robot, goal_x, goal_y, flag) do

    right = if x >= @table_top_x do
      100
    else
      sol2(%ToyRobot.Position{robot | facing: :east},x+1, Map.get(@robot_map_y_atom_to_num, y), goal_x, goal_y, flag)
    end

    left = if x <= 1 do
      100
    else
      sol2(%ToyRobot.Position{robot | facing: :west},x-1, Map.get(@robot_map_y_atom_to_num, y), goal_x, goal_y, flag)
    end

    forward = if y >= @table_top_y do
      100
    else
      sol2(robot, x, (Map.get(@robot_map_y_atom_to_num, y) + 1), goal_x, goal_y, flag)
    end

    flag = cond do
      ((left == forward) and (right == forward)) ->
        Map.put(flag,%ToyRobot.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)}, 100)
      true -> flag
    end

    {forward, right, left, flag}
  end

  def sol2(%ToyRobot.Position{x: x, y: y, facing: :east} = robot, goal_x, goal_y, flag) do

    right = if y <= :a do
      100
    else
      sol2(%ToyRobot.Position{robot | facing: :south}, x, (Map.get(@robot_map_y_atom_to_num, y) - 1), goal_x, goal_y, flag)
    end

    left = if y >= @table_top_y do
      100
    else
      sol2(%ToyRobot.Position{robot | facing: :north}, x, (Map.get(@robot_map_y_atom_to_num, y) + 1), goal_x, goal_y, flag)
    end

    forward = if x >= @table_top_x do
      100
    else
      sol2(robot, x+1, Map.get(@robot_map_y_atom_to_num, y), goal_x, goal_y, flag)
    end

    flag = cond do
      ((left == forward) and (right == forward)) ->
          flag = Map.put(flag,%ToyRobot.Position{robot | x: x-1, facing: :east}, 100)
          flag
      true -> flag
    end

    {forward, right, left, flag}
  end

  def sol2(%ToyRobot.Position{x: x, y: y, facing: :south} = robot, goal_x, goal_y, flag) do
    right = if x <= 1 do
      100
    else
      sol2(%ToyRobot.Position{robot | facing: :west}, x-1, Map.get(@robot_map_y_atom_to_num, y), goal_x, goal_y, flag)
    end

    left = if x >= @table_top_x do
      100
    else
      sol2(%ToyRobot.Position{robot | facing: :east}, x+1, Map.get(@robot_map_y_atom_to_num, y), goal_x, goal_y, flag)
    end

    forward = if y <= :a do
      100
    else
      sol2(robot, x, (Map.get(@robot_map_y_atom_to_num, y) - 1), goal_x, goal_y, flag)
    end

    flag = cond do
      ((left == forward) and (right == forward)) ->
        Map.put(flag,%ToyRobot.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)}, 100)
      true -> flag
    end

    {forward, right, left, flag}

  end

  def sol2(%ToyRobot.Position{x: x, y: y, facing: :west} = robot, goal_x, goal_y, flag) do

    right = if y >= @table_top_y do
      100
    else
      sol2(%ToyRobot.Position{robot | facing: :north}, x, (Map.get(@robot_map_y_atom_to_num, y) + 1), goal_x, goal_y, flag)
    end

    left = if y <= :a do
      100
    else
      sol2(%ToyRobot.Position{robot | facing: :south}, x, (Map.get(@robot_map_y_atom_to_num, y) - 1), goal_x, goal_y, flag)
    end

    forward = if x <= 1 do
      100
    else
      sol2(robot, x-1, Map.get(@robot_map_y_atom_to_num, y), goal_x, goal_y, flag)
    end

    {x, y, facing} = report(robot)
    flag = cond do
      ((left == forward) and (right == forward)) ->
        flag = Map.put(flag,%ToyRobot.Position{x: x+1, y: y, facing: facing}, 100)
        flag
      true -> flag
    end

    {forward, right, left, flag}

  end

  def sol2(robot, x, y, goal_x, goal_y, flag) do

    dist = if Map.has_key?(flag, robot) do
      {:ok,dist} = Map.fetch(flag, robot)
      dist

    else
      abs(x - goal_x) + abs(y - Map.get(@robot_map_y_atom_to_num, goal_y))
    end

    dist

  end

  def cost_solution(robot, goal_x, goal_y, is_obs_ahead, flag) do
    {forward, left, right, flag} =

      if is_obs_ahead do
        {_, right, left, flag} = sol2(robot, goal_x, goal_y, flag)
        {100, left, right, flag}
      else
        {forward, right, left, flag} = sol2(robot, goal_x, goal_y, flag)
        {forward, left, right, flag}
      end

    {forward, left, right, flag}

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
end
