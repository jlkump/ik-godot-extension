extends Control


func _on_quit_pressed():
	get_tree().quit()

func _on_continue_pressed():
	GameManager.get_singleton().toggle_pause()
