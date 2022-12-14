defmodule LineFollower do
  @moduledoc """
  Documentation for `FW_DEMO`.

  Different functions provided for testing components of Alpha Bot.
  test_wlf_sensors  - to test white line sensors
  test_motion       - to test motion of the Robot
  """


  require Logger
  use Bitwise
  alias Circuits.GPIO

  @sensor_pins [cs: 5, clock: 25, address: 24, dataout: 23]
  @ir_pins [dr: 16, dl: 19]
  @motor_pins [lf: 12, lb: 13, rf: 20, rb: 21]
  @pwm_pins [enl: 6, enr: 26]

  @ref_atoms [:cs, :clock, :address, :dataout]
  @lf_sensor_data %{sensor0: 0, sensor1: 0, sensor2: 0, sensor3: 0, sensor4: 0, sensor5: 0}
  @lf_sensor_map %{0 => :sensor0, 1 => :sensor1, 2 => :sensor2, 3 => :sensor3, 4 => :sensor4, 5 => :sensor5}

  @forward [0, 1, 1, 0]
  @backward [1, 0, 0, 1]
  @left [0, 1, 0, 0]
  @right [0, 0, 1, 0]
  @stop [0, 0, 0, 0]

  @duty_cycles [150, 70, 0]
  @pwm_frequency 50

  
  @doc """
  Tests motion of the Robot

  Example:

      iex> FW_DEMO.test_motion
      :ok

  Note: On executing above function Robot will move forward, backward, left, right
  for 500ms each and then stops
  """
  
  def driver() do
  	steps = [1, 2, 1, 1]
  	checker(steps)

  end

  def for_right() do
  	Logger.debug("Turning right")
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)
    motion_list = [@forward,@stop]
    Enum.each(motion_list, fn motion -> motor_action_delay(motor_ref,motion, 90) end)
    motion_list = [@right,@stop]
    Enum.each(motion_list, fn motion -> motor_action_delay(motor_ref,motion, 450) end)
  end

   def for_left() do
  	Logger.debug("Turning left")
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)
    motion_list = [@forward,@stop]
    Enum.each(motion_list, fn motion -> motor_action_delay(motor_ref,motion, 90) end)
    motion_list = [@left,@stop]
    Enum.each(motion_list, fn motion -> motor_action_delay(motor_ref,motion, 450) end)
  end

  def for_forward() do
    Logger.debug("Moving Forward")
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)

    lf_data = test_wlf_sensors()

    cond do
		(Enum.at(lf_data, 3) >= 800)
		->	
    		motion_list = [@forward,@stop]
    		Enum.each(motion_list, fn motion -> motor_action_delay(motor_ref,motion,40) end)

    	(Enum.at(lf_data, 3) < 800) and (Enum.at(lf_data, 2) >= 800) and (Enum.at(lf_data, 4) < 800) and 
		(Enum.at(lf_data, 1) < 800) and (Enum.at(lf_data, 5) < 800)
		->	
    		motion_list = [@left,@stop]
    		Enum.each(motion_list, fn motion -> motor_action_delay(motor_ref,motion,40) end)

    	(Enum.at(lf_data, 3) < 800) and (Enum.at(lf_data, 2) < 800) and (Enum.at(lf_data, 4) < 800) and 
		(Enum.at(lf_data, 1) >= 800) and (Enum.at(lf_data, 5) < 800)
		->	
    		motion_list = [@left,@stop]
    		Enum.each(motion_list, fn motion -> motor_action_delay(motor_ref,motion,40) end)

    	(Enum.at(lf_data, 3) < 800) and (Enum.at(lf_data, 2) < 800) and (Enum.at(lf_data, 4) >= 800) and 
		(Enum.at(lf_data, 1) < 800) and (Enum.at(lf_data, 5) < 800)
		->	
    		motion_list = [@right,@stop]
    		Enum.each(motion_list, fn motion -> motor_action_delay(motor_ref,motion,40) end)

    	(Enum.at(lf_data, 3) < 800) and (Enum.at(lf_data, 2) < 800) and (Enum.at(lf_data, 4) < 800) and 
		(Enum.at(lf_data, 1) < 800) and (Enum.at(lf_data, 5) >= 800)
		->	
    		motion_list = [@right,@stop]
    		Enum.each(motion_list, fn motion -> motor_action_delay(motor_ref,motion,40) end)

    	true -> IO.puts("Align Properly")

	end
  end

  def checker(move_list) do
    Logger.debug("Testing Motion of the Robot ")
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)

    lf_data = test_wlf_sensors()
    IO.inspect(lf_data)

	move_list = if ((Enum.at(lf_data, 1) >= 800) and (Enum.at(lf_data, 2) >= 800) and (Enum.at(lf_data, 3) >= 800)) or
		((Enum.at(lf_data, 3) >= 800) and (Enum.at(lf_data, 4) >= 800) and (Enum.at(lf_data, 5) >= 800)) do

		IO.puts("Reached Node")
		curr_move = hd(move_list)
		IO.inspect(move_list)
		move_list = List.delete_at(move_list, 0)

		cond do
			curr_move == 2 -> 
				IO.puts("Need to turn right")
				for_right()

			curr_move == 3 -> 
				IO.puts("Need to turn left")
				for_left()

			curr_move == 1 ->
				IO.puts("Need to move forward")
				for_forward()
			true ->
				IO.puts("Invalid move")
		end
		move_list
	else
		for_forward()
		move_list
	end

	if length(move_list) != 0 do
		checker(move_list)
	end
  end

  @doc """
  Tests white line sensor modules reading

  Example:

      iex> FW_DEMO.test_wlf_sensors
      [0, 958, 851, 969, 975, 943]  // on white surface
      [0, 449, 356, 312, 321, 267]  // on black surface
  """
  def test_wlf_sensors do
    Logger.debug("Testing white line sensors connected ")
    sensor_ref = Enum.map(@sensor_pins, fn {atom, pin_no} -> configure_sensor({atom, pin_no}) end)
    sensor_ref = Enum.map(sensor_ref, fn{_atom, ref_id} -> ref_id end)
    sensor_ref = Enum.zip(@ref_atoms, sensor_ref)
    get_lfa_readings([0,1,2,3,4,5], sensor_ref)
  end


  @doc """
  Supporting function for test_wlf_sensors
  Configures sensor pins as input or output

  [cs: output, clock: output, address: output, dataout: input]
  """
  defp configure_sensor({atom, pin_no}) do
    if (atom == :dataout) do
      GPIO.open(pin_no, :input, pull_mode: :pullup)
    else
      GPIO.open(pin_no, :output)
    end
  end

  @doc """
  Supporting function for test_wlf_sensors
  Reads the sensor values into an array. "sensor_list" is used to provide list
  of the sesnors for which readings are needed


  The values returned are a measure of the reflectance in abstract units,
  with higher values corresponding to lower reflectance (e.g. a black
  surface or void)
  """
  defp get_lfa_readings(sensor_list, sensor_ref) do

  	Enum.each(0..5, fn n -> provide_clock(sensor_ref) end)
    GPIO.write(sensor_ref[:cs], 1)
    append_sensor_list = sensor_list
    temp_sensor_list = append_sensor_list

    #IO.inspect(append_sensor_list)
    #IO.inspect(temp_sensor_list)
    
    append_sensor_list
        |> Enum.with_index
        |> Enum.map(fn {sens_num, sens_idx} ->
              analog_read(sens_num, sensor_ref, Enum.fetch(temp_sensor_list, sens_idx))
              end)
  end

  @doc """
  Supporting function for test_wlf_sensors
  """
  defp analog_read(sens_num, sensor_ref, {_, sensor_atom_num}) do

    GPIO.write(sensor_ref[:cs], 0)
    %{^sensor_atom_num => sensor_atom} = @lf_sensor_map
    Enum.reduce(0..9, @lf_sensor_data, fn n, acc ->
                                          read_data(n, acc, sens_num, sensor_ref, sensor_atom_num)
                                          |> clock_signal(n, sensor_ref)
                                        end)[sensor_atom]
  end

  @doc """
  Supporting function for test_wlf_sensors
  """
  defp read_data(n, acc, sens_num, sensor_ref, sensor_atom_num) do
    if (n < 4) do

      if (((sens_num) >>> (3 - n)) &&& 0x01) == 1 do
        GPIO.write(sensor_ref[:address], 1)
      else
        GPIO.write(sensor_ref[:address], 0)
      end
      Process.sleep(1)
    end

    %{^sensor_atom_num => sensor_atom} = @lf_sensor_map
    if (n <= 9) do
      Map.update!(acc, sensor_atom, fn sensor_atom -> ( sensor_atom <<< 1 ||| GPIO.read(sensor_ref[:dataout]) ) end)
    end
  end

  @doc """
  Supporting function for test_wlf_sensors used for providing clock pulses
  """
  defp provide_clock(sensor_ref) do
    GPIO.write(sensor_ref[:clock], 1)
    GPIO.write(sensor_ref[:clock], 0)
  end

  @doc """
  Supporting function for test_wlf_sensors used for providing clock pulses
  """
  defp clock_signal(acc, n, sensor_ref) do
    GPIO.write(sensor_ref[:clock], 1)
    GPIO.write(sensor_ref[:clock], 0)
    acc
  end

  @doc """
  Supporting function for test_motion
  """
  defp motor_action(motor_ref,motion) do
    motor_ref |> Enum.zip(motion) |> Enum.each(fn {{_, ref_no}, value} -> GPIO.write(ref_no, value) end)
    Process.sleep(300)
  end

  defp motor_action_delay(motor_ref,motion, delay) do
    motor_ref |> Enum.zip(motion) |> Enum.each(fn {{_, ref_no}, value} -> GPIO.write(ref_no, value) end)
    Process.sleep(delay)
  end

end
