extends Node

var player_character = null

func _ready():
	spawn_complete_walker()
	player_character.set_global_position($spawn.get_global_position())
	pass

func _input(event):
	if event.is_action_pressed("character_walker"):
		$player_cam.set_focus_object_path(NodePath())
		player_character.queue_free()
		spawn_simple_walker()
		# instantiate walker
		pass
	if event.is_action_pressed("character_hopper"):
		print("Character hopper")
		$player_cam.set_focus_object_path(NodePath())
		player_character.queue_free()
		spawn_hopper()
		# instantiate walker
		pass
	if event.is_action_pressed("character_complete_walker"):
		$player_cam.set_focus_object_path(NodePath())
		player_character.queue_free()
		spawn_complete_walker()

func spawn_simple_walker():
	pass

func spawn_hopper():
	player_character = preload("res://scenes/prefabs/biped_walker.tscn").instantiate()
	if (player_character as PlayerController3D):
		add_child(player_character)
		(player_character as PlayerController3D).set_camera_controller_path($player_cam.get_path())
		$player_cam.set_focus_object_path(player_character.get_path())
		var ik_con = (player_character as PlayerController3D).get_ik_con_obj()
		if (ik_con as InverseKinematicController):
			print("Got ik con")
			var ik_chains = ik_con.get_ik_chain_objs()
			for i in ik_chains.size():
				var target_paths = $targets/hopper.get_children()
				if (ik_chains[i] as InverseKinematicChain):
					print("Got ik chain")
					(ik_chains[i] as InverseKinematicChain).set_target_pos_path(target_paths[i].get_path())
		else:
			printerr("didnt get ik con")

func spawn_complete_walker():
	player_character = preload("res://scenes/prefabs/complete_walker.tscn").instantiate()
	if (player_character as PlayerController3D):
		add_child(player_character)
		(player_character as PlayerController3D).set_camera_controller_path($player_cam.get_path())
		$player_cam.set_focus_object_path(player_character.get_path())
		var ik_con = (player_character as PlayerController3D).get_ik_con_obj()
		if (ik_con as InverseKinematicController):
			print("Got ik con")
			var ik_chains = ik_con.get_ik_chain_objs()
			for i in ik_chains.size():
				var target_paths = $targets/complete_walker.get_children()
				if (ik_chains[i] as InverseKinematicChain):
					print("Got ik chain")
					(ik_chains[i] as InverseKinematicChain).set_target_pos_path(target_paths[i].get_path())
		else:
			printerr("didnt get ik con")
