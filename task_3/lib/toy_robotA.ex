defmodule CLI.ToyRobotA do
  # max x-coordinate of table top
  @table_top_x 5
  # max y-coordinate of table top
  @table_top_y :e
  # mapping of y-coordinates
  @robot_map_y_atom_to_num %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5}

  @robot_B %{:x=>1,:y=>:e}

  @doc """
  Places the robot to the default position of (1, A, North)

  Examples:

      iex> CLI.ToyRobotA.place
      {:ok, %CLI.Position{facing: :north, x: 1, y: :a}}
  """
  def place do
    {:ok, %CLI.Position{}}
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

      iex> CLI.ToyRobotA.place(1, :b, :south)
      {:ok, %CLI.Position{facing: :south, x: 1, y: :b}}

      iex> CLI.ToyRobotA.place(-1, :f, :north)
      {:failure, "Invalid position"}

      iex> CLI.ToyRobotA.place(3, :c, :north_east)
      {:failure, "Invalid facing direction"}
  """
  def place(x, y, facing) do
    # IO.puts String.upcase("A I'm placed at => #{x},#{y},#{facing}")
    {:ok, %CLI.Position{x: x, y: y, facing: facing}}
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

  def stop(_robot, goal_x, goal_y, _cli_proc_name) when goal_x < 1 or goal_y < :a or goal_x > @table_top_x or goal_y > @table_top_y do
    {:failure, "Invalid STOP position"}
  end

  @doc """
  Provide GOAL positions to the robot as given location of [(x1, y1),(x2, y2),..] and plan the path from START to these locations.
  Passing the CLI Server process name that will be used to send robot's current status after each action is taken.
  Spawn a process and register it with name ':client_toyrobotA' which is used by CLI Server to send an
  indication for the presence of obstacle ahead of robot's current position and facing.
  """
  def stop(robot, goal_locs, cli_proc_name) do
    ###########################
    ## complete this funcion ##
    ###########################

    ## A acts as master for some important decisions

    parent = self()
    Process.flag(:trap_exit,true)
    pid = spawn_link(fn -> controller_of_A(parent) end)
    Process.register(pid, :client_toyrobotA)


    {:ok,state_a} = Agent.start_link( fn -> %{} end )

    states =  %{
      robot: robot,   # to access robot's current position to avoid crash
      cli: cli_proc_name,
      curr_goal: [] , # to keep track of current goal ( after calculating next best goal )
      flag: %{},  # used in path_plan_x_y
      is_obs: send_robot_status(robot, cli_proc_name), # used in path_plan_x_y
      done: false,
      reported_a: false,
      reported_b: false
    }

    Agent.update(state_a,
      fn state -> Map.merge(state,states) end
    )

    Process.register(state_a,:state_a)

    # register an agent for goals left
    {:ok , goals_left} = Agent.start_link(fn -> goal_locs end)
    Process.register(goals_left,:goals_left)


    # wait for B to get registered
    send(:client_toyrobotA,{:wait,robot})


    {:ok,robot,_roboB} = get_after_done()
    #IO.puts("A")
    #IO.inspect(robot)
    {:ok, robot}
  end


  def get_after_done() do

    {:ok,robotA,robotB} =

    if Process.whereis(:state_a) == nil or Process.whereis(:state_b) == nil  do
        {:ok,robotA,robotB}  = get_after_done()
        {:ok,robotA,robotB}
    else
      a_done = Agent.get(:state_a,fn s->s end) |> Map.get(:done)
      b_done = Agent.get(:state_b,fn s->s end) |> Map.get(:done)

      {:ok,robotA,robotB}  =
      if a_done and b_done do
        robotA  = Agent.get(:state_a,fn s->s end) |> Map.get(:robot,:none)
        robotB  = Agent.get(:state_b,fn s->s end) |> Map.get(:robot,:none)

        Agent.update(:state_a , fn state ->
          state |> Map.put(:reported_a,true)
        end)

        {:ok,robotA,robotB}
      else
        {:ok,robotA,robotB}  = get_after_done()
        {:ok,robotA,robotB}
      end

      {:ok,robotA,robotB}
    end

    {:ok,robotA,robotB}


  end

  # declared outside now
  def heuristic(x,y,goal_x,goal_y) do

    y = @robot_map_y_atom_to_num[y]
    goal_y = @robot_map_y_atom_to_num[goal_y]
    d = (goal_y-y)*(goal_y-y) + (goal_x-x)*(goal_x-x)
    :math.sqrt(d)

  end


  def find_nearest_goal(goal_locs, currx, curry) do

    nearest =
      if Enum.count(goal_locs) != 0 do

        nearest = goal_locs |> Enum.min_by(
                    fn d ->
                      gx = d |> Enum.at(0) |> String.to_integer
                      gy = d |> Enum.at(1)  |> String.to_atom
                      heuristic(currx,curry,gx,gy)
                    end )

        nearest = [ nearest |> Enum.at(0) |> String.to_integer,
                    nearest |> Enum.at(1)  |> String.to_atom
                  ]

        nearest
      else
        []
      end

      nearest
  end


  def controller_of_A(parent) do
    receive do
      {:obstacle_presence, is_obs_ahead} -> send(parent, {:obstacle_presence, is_obs_ahead})

      {:wait,robot} ->
        if Process.whereis(:state_b) == nil do
          #IO.puts("Waiting")
          send(self(),{:wait,robot})
        else

          goal_locs = Agent.get(:goals_left,fn state -> state  end)

          if Enum.count(goal_locs) == 1 do
            #IO.puts("Only one goal")
            roboB = Agent.get(:state_b,fn state-> state end) |> Map.get(:robot,:none)
            {xa,ya,_facing} = report(robot)
            {xb,yb,_facing} = report(roboB)

            goal = goal_locs |> Enum.at(0)
            gx = goal |> Enum.at(0) |> String.to_integer
            gy = goal |> Enum.at(1)  |> String.to_atom

            if heuristic(xa,ya,gx,gy) < heuristic(xb,yb,gx,gy) do
              #IO.puts("A is nearest to goal")
              # update first goal of A
              send(:client_toyrobotA,{:update_curr_goal,robot})
              # update first goal of B
              roboB = Agent.get(:state_b,fn state-> state end) |> Map.get(:robot,:none)
              Process.send_after(:client_toyrobotB,{:update_curr_goal,roboB},100)
              # call to execute first move of A
              send(:client_toyrobotA,{:make_next_move_A})
            else
              #IO.puts("B is nearest to goal")
              # update first goal of B
              send(:client_toyrobotB,{:update_curr_goal,robot})
              # update first goal of A
              roboB = Agent.get(:state_b,fn state-> state end) |> Map.get(:robot,:none)
              Process.send_after(:client_toyrobotA,{:update_curr_goal,roboB},100)
              #send(:client_toyrobotA,{:update_curr_goal,roboB})
              # call to execute first move of A
              send(:client_toyrobotA,{:make_next_move_A})

            end

            #Process.sleep(20000)

          else
            # update first goal of A
            send(:client_toyrobotA,{:update_curr_goal,robot})
            # update first goal of B
            roboB = Agent.get(:state_b,fn state-> state end) |> Map.get(:robot,:none)
            send(:client_toyrobotB,{:update_curr_goal,roboB})
            # call to execute first move of A
            send(:client_toyrobotA,{:make_next_move_A})
          end

        end




      # to update current goal
      {:update_curr_goal,robot} ->


        {x,y,_faicng} = report(robot)

        goal_locs = Agent.get(:goals_left,fn state -> state  end)

        # find all the nearest goal from all the locs left
        nearest_goal = find_nearest_goal(goal_locs,x,y)

        goal_of_B = Agent.get(:state_b,fn state-> state end) |> Map.get(:curr_goal)

        # if nearest goal == goal of B then don't do anything
        if nearest_goal != goal_of_B and nearest_goal != []  do

          goal_x =  nearest_goal |> Enum.at(0)
          goal_y =  nearest_goal |> Enum.at(1)

          remove = [ Integer.to_string(goal_x),Atom.to_string(goal_y) ]

          Agent.update(:state_a , fn state ->
            state |> Map.put(:curr_goal,[goal_x,goal_y])
          end)

          # remove that goal from goals_left
          Agent.get_and_update(:goals_left, fn state ->
            {state , state |> Enum.reject(fn g-> g == remove end )}
          end)

        end

      {:delay} ->
        #If A understands that all reporting has been done it ends delay
        reportedA = Agent.get(:state_a,fn s->s end) |> Map.get(:reported_a,:none)
        reportedB = Agent.get(:state_a,fn s->s end) |> Map.get(:reported_b,:none)

        if [reportedA,reportedB] != [true,true]  do
          send(:client_toyrobotB,{:delay})
        end

      # move A
      {:make_next_move_A} ->

        #IO.puts("move A")
        # get all required, updated values from state_a
        states = Agent.get(:state_a , fn state -> state  end)
        robot = states |> Map.get(:robot,:none)
        goal = states |> Map.get(:curr_goal,:none)
        cli = states |> Map.get(:cli,:none)
        flag = states |> Map.get(:flag,:none)
        is_obs = states |> Map.get(:is_obs,:none)

        #IO.puts("A curr goal")
        #IO.inspect(goal)

        if goal != [] do
          #IO.puts("A move")
          #IO.inspect(goal)

          [gx,gy] = goal
          # perform one move
          path_plan_x_y(robot,gx,gy,is_obs,cli,flag)

        else
          # do we have to do this really?
          #send_robot_status(robot,cli)
          #send(self(),{:reached_goal,robot})

          #{gx,gy,_facing} = report(robot)
          #IO.puts("A #{gx},#{gy}")
          #Process.sleep(1000)
          #path_plan_x_y(robot,gx,gy,is_obs,cli,flag)

        end

        # check if a and b both are done then dont ask to move


        # if both b and a are done then exit
        b_done = Agent.get(:state_b, fn s->s end) |> Map.get(:done,nil)
        a_done = Agent.get(:state_a, fn s->s end) |> Map.get(:done,nil)



        if b_done and a_done do
          #IO.puts("Done from A")
          send(:client_toyrobotB,{:delay} )
        else

          if b_done do
            send(:client_toyrobotA,{:make_next_move_A})
          else
            send(:client_toyrobotB,{:make_next_move_B})
          end
        end




      # if reached goal, update current goal
      {:reached_goal,robot}->

        goal_locs = Agent.get(:goals_left,fn state -> state  end)

        if Enum.count(goal_locs) != 0 do

          send(:client_toyrobotA,{:update_curr_goal,robot})

        else

          Agent.update(:state_a , fn state ->
            state |> Map.merge(%{:done=>true})
          end)

        end

    end
    controller_of_A(parent)
  end


  def rev_map(y) do
    m = %{1=>:a , 2=>:b , 3=>:c , 4=>:d , 5=>:e }
    mapped_y = m |> Map.get(y)
    mapped_y
  end


  def find_next_pos(x,y,facing) do
    {:ok,nextx,nexty} =
    cond do
      facing == :north ->
        #code
        {:ok,x,@robot_map_y_atom_to_num[y]+1 |> rev_map }

        facing == :south ->
        #code
        {:ok,x,@robot_map_y_atom_to_num[y]-1 |> rev_map }

      facing == :east ->
        #code
        {:ok,x+1,y}

      facing == :west ->
        #code
        {:ok,x-1,y}

    end
    {:ok,nextx,nexty}
  end

  def get_obstacle(robot, cli_proc_name) do
    roboB = Agent.get(:state_b , fn state->state end) |> Map.get(:robot,:none)
    {xb,yb,bfacing} = report(roboB)

    {x,y,facing} = report(robot)

    bool = send_robot_status(robot, cli_proc_name)

    bool = cond do
      x == xb and
      ((@robot_map_y_atom_to_num[y] == @robot_map_y_atom_to_num[yb]+1 and facing == :south and bfacing == :north) or
      (@robot_map_y_atom_to_num[y] == @robot_map_y_atom_to_num[yb]-1 and facing == :north and bfacing == :south))
      ->
        bool = true
        bool
      y == yb and
      ((x == xb+1 and facing == :west and bfacing == :east) or
      (x == xb-1 and bfacing == :west and facing == :east))
      ->
        bool = true
        bool
      true ->
        bool
    end
    bool
  end

  def path_plan_x_y(robot, goal_x, goal_y, is_obs_ahead, cli_proc_name, flag) do

    {x,y,facing} = report(robot)


    flag = cond do
      ((is_obs_ahead) and (not Map.has_key?(flag, robot))) -> Map.put(flag, robot, 100)
      true -> flag
    end

    {:ok, robot,is_obs_ahead,flag} = cond do

      ((x == goal_x) and (y == goal_y)) ->

        #IO.puts("A reached #{x}, #{y}\n")
        '''
        if Enum.member?(Process.registered(),:client_toyrobotA) do
          send_robot_status(robot, cli_proc_name)
        end
        '''
        send(:client_toyrobotA,{:reached_goal,robot})
        {:ok, robot,is_obs_ahead,flag}

      true ->

          {forward, left, right, flag} = cost_solution(robot, goal_x, goal_y, is_obs_ahead, flag)

          {robot, flag, is_obs_ahead} = cond do

            ((forward <= right) and (forward <= left) and (forward != 100)) ->
              # to check if B is in front and avoid if it is

              roboB = Agent.get(:state_b , fn state->state end) |> Map.get(:robot,:none)
              {xb,yb,_facing} = report(roboB)

              {:ok,next_x,next_y} = find_next_pos(x,y,facing)

              robot =
              if {xb,yb} != {next_x,next_y} do
                robot = move(robot)
                robot
              else
                robot
              end

              is_obs_ahead = get_obstacle(robot, cli_proc_name)
              {robot, flag, is_obs_ahead}

            ((right <= left) and (right != 100)) ->
              robot = right(robot)
              is_obs_ahead = send_robot_status(robot, cli_proc_name)
              {robot, flag, is_obs_ahead}

            ((right >= left) and (left != 100)) ->
              robot = left(robot)
              is_obs_ahead = send_robot_status(robot, cli_proc_name)
              {robot, flag, is_obs_ahead}

            true ->
              robot = right(robot)
              _is_obs_ahead = send_robot_status(robot, cli_proc_name)
              robot = right(robot)
              is_obs_ahead = send_robot_status(robot, cli_proc_name)
              {robot,flag,is_obs_ahead}

          end
          #{:ok, robot} = path_plan_x_y(robot, goal_x, goal_y, is_obs_ahead, cli_proc_name, flag)
          {:ok,robot,is_obs_ahead,flag}
    end

    #{:ok, robot,is_obs_ahead,flag}
    # updating all the required states of a
    Agent.update(:state_a , fn state ->

      state |> Map.merge(
        %{:robot=>robot,:is_obs=>is_obs_ahead,:flag=>flag}
      )

    end)

    #IO.puts("After path A")
    #IO.inspect(Agent.get(:state_a,fn state-> state end))

  end

  def sol2(%CLI.Position{x: x, y: y, facing: :north} = robot, goal_x, goal_y, flag) do

    right = if x >= @table_top_x do
      100
    else
      sol2(%CLI.Position{robot | facing: :east},x+1, Map.get(@robot_map_y_atom_to_num, y), goal_x, goal_y, flag)
    end

    left = if x <= 1 do
      100
    else
      sol2(%CLI.Position{robot | facing: :west},x-1, Map.get(@robot_map_y_atom_to_num, y), goal_x, goal_y, flag)
    end

    forward = if y >= @table_top_y do
      100
    else
      sol2(robot, x, (Map.get(@robot_map_y_atom_to_num, y) + 1), goal_x, goal_y, flag)
    end

    flag = cond do
      ((left == forward) and (right == forward)) ->
        Map.put(flag,%CLI.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)}, 100)
      true -> flag
    end

    {forward, right, left, flag}
  end

  def sol2(%CLI.Position{x: x, y: y, facing: :east} = robot, goal_x, goal_y, flag) do

    right = if y <= :a do
      100
    else
      sol2(%CLI.Position{robot | facing: :south}, x, (Map.get(@robot_map_y_atom_to_num, y) - 1), goal_x, goal_y, flag)
    end

    left = if y >= @table_top_y do
      100
    else
      sol2(%CLI.Position{robot | facing: :north}, x, (Map.get(@robot_map_y_atom_to_num, y) + 1), goal_x, goal_y, flag)
    end

    forward = if x >= @table_top_x do
      100
    else
      sol2(robot, x+1, Map.get(@robot_map_y_atom_to_num, y), goal_x, goal_y, flag)
    end

    flag = cond do
      ((left == forward) and (right == forward)) ->
          flag = Map.put(flag,%CLI.Position{robot | x: x-1, facing: :east}, 100)
          flag
      true -> flag
    end

    {forward, right, left, flag}
  end

  def sol2(%CLI.Position{x: x, y: y, facing: :south} = robot, goal_x, goal_y, flag) do
    right = if x <= 1 do
      100
    else
      sol2(%CLI.Position{robot | facing: :west}, x-1, Map.get(@robot_map_y_atom_to_num, y), goal_x, goal_y, flag)
    end

    left = if x >= @table_top_x do
      100
    else
      sol2(%CLI.Position{robot | facing: :east}, x+1, Map.get(@robot_map_y_atom_to_num, y), goal_x, goal_y, flag)
    end

    forward = if y <= :a do
      100
    else
      sol2(robot, x, (Map.get(@robot_map_y_atom_to_num, y) - 1), goal_x, goal_y, flag)
    end

    flag = cond do
      ((left == forward) and (right == forward)) ->
        Map.put(flag,%CLI.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)}, 100)
      true -> flag
    end

    {forward, right, left, flag}

  end

  def sol2(%CLI.Position{x: x, y: y, facing: :west} = robot, goal_x, goal_y, flag) do

    right = if y >= @table_top_y do
      100
    else
      sol2(%CLI.Position{robot | facing: :north}, x, (Map.get(@robot_map_y_atom_to_num, y) + 1), goal_x, goal_y, flag)
    end

    left = if y <= :a do
      100
    else
      sol2(%CLI.Position{robot | facing: :south}, x, (Map.get(@robot_map_y_atom_to_num, y) - 1), goal_x, goal_y, flag)
    end

    forward = if x <= 1 do
      100
    else
      sol2(robot, x-1, Map.get(@robot_map_y_atom_to_num, y), goal_x, goal_y, flag)
    end

    {x, y, facing} = report(robot)
    flag = cond do
      ((left == forward) and (right == forward)) ->
        flag = Map.put(flag,%CLI.Position{x: x+1, y: y, facing: facing}, 100)
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
  Send Toy Robot's current status i.e. location (x, y) and facing
  to the CLI Server process after each action is taken.
  Listen to the CLI Server and wait for the message indicating the presence of obstacle.
  The message with the format: '{:obstacle_presence, < true or false >}'.
  """
  def send_robot_status(%CLI.Position{x: x, y: y, facing: facing} = _robot, cli_proc_name) do
    send(cli_proc_name, {:toyrobotA_status, x, y, facing})
    # IO.puts("Sent by Toy Robot Client: #{x}, #{y}, #{facing}")
    listen_from_server()
  end

  @doc """
  Listen to the CLI Server and wait for the message indicating the presence of obstacle.
  The message with the format: '{:obstacle_presence, < true or false >}'.
  """
  def listen_from_server() do
    receive do
      {:obstacle_presence, is_obs_ahead} ->
        is_obs_ahead
    end
  end

  @doc """
  Provides the report of the robot's current position

  Examples:

      iex> {:ok, robot} = CLI.ToyRobotA.place(2, :b, :west)
      iex> CLI.ToyRobotA.report(robot)
      {2, :b, :west}
  """
  def report(%CLI.Position{x: x, y: y, facing: facing} = _robot) do
    {x, y, facing}
  end

  @directions_to_the_right %{north: :east, east: :south, south: :west, west: :north}
  @doc """
  Rotates the robot to the right
  """
  def right(%CLI.Position{facing: facing} = robot) do
    %CLI.Position{robot | facing: @directions_to_the_right[facing]}
  end

  @directions_to_the_left Enum.map(@directions_to_the_right, fn {from, to} -> {to, from} end)
  @doc """
  Rotates the robot to the left
  """
  def left(%CLI.Position{facing: facing} = robot) do
    %CLI.Position{robot | facing: @directions_to_the_left[facing]}
  end

  @doc """
  Moves the robot to the north, but prevents it to fall
  """
  def move(%CLI.Position{x: _, y: y, facing: :north} = robot) when y < @table_top_y do
    %CLI.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the east, but prevents it to fall
  """
  def move(%CLI.Position{x: x, y: _, facing: :east} = robot) when x < @table_top_x do
    %CLI.Position{robot | x: x + 1}
  end

  @doc """
  Moves the robot to the south, but prevents it to fall
  """
  def move(%CLI.Position{x: _, y: y, facing: :south} = robot) when y > :a do
    %CLI.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the west, but prevents it to fall
  """
  def move(%CLI.Position{x: x, y: _, facing: :west} = robot) when x > 1 do
    %CLI.Position{robot | x: x - 1}
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
