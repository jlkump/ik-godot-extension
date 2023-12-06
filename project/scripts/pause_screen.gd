extends Control

signal restart_level_pressed

func _on_quit_pressed():
	get_tree().quit()

func _on_continue_pressed():
	GameManager.get_singleton().toggle_pause()


func _on_restart_pressed():
	emit_signal("restart_level_pressed")
