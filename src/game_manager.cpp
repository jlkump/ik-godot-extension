#include "game_manager.h"

#include <godot_cpp/classes/window.hpp>

// Utility includes
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

GameManager* GameManager::instance;

void GameManager::_bind_methods() {
    ADD_SIGNAL(MethodInfo("game_pause", PropertyInfo(Variant::BOOL, "is_game_paused")));
}

GameManager::GameManager() : game_state_(PLAYING) {
    GameManager::instance = nullptr;
}

GameManager::~GameManager() {}

bool GameManager::is_valid_state(State s) {
    switch (s) {
        case PAUSED:
            return game_state_ == PLAYING;
        case PLAYING:
            return true;
        case MENU:
            return true;
    }
    return false;
}

GameManager* GameManager::get_singleton() {
    if (instance != nullptr && instance->is_node_ready()) {
        return instance;
    } else {
        return nullptr;
    }
}

void GameManager::set_game_state(State s) {
    switch (game_state_) {
        case PAUSED:
            if (s == PLAYING) {
                if (Input::get_singleton() != nullptr) {
                    Input::get_singleton()->set_mouse_mode(Input::MOUSE_MODE_CONFINED_HIDDEN);
                }
            }
        case PLAYING:
            if (s == MENU || s == PAUSED) {
                if (Input::get_singleton() != nullptr) {
                    Input::get_singleton()->set_mouse_mode(Input::MOUSE_MODE_VISIBLE);
                }
            }
    }
    game_state_ = s;
}

void GameManager::toggle_pause() {
    if (game_state_ == PAUSED) {
        set_game_state(PLAYING);
        emit_signal("game_pause", false);
    } else {
        set_game_state(PAUSED);
        emit_signal("game_pause", true);
    }
}

void GameManager::_ready() {
    instance = this;
    if (Engine::get_singleton() != nullptr && !Engine::get_singleton()->is_editor_hint()) {
        set_game_state(game_state_);
    }
}

void GameManager::_process(double delta) {

}

void GameManager::_input(const Ref<InputEvent>& event) {
    if (!event.is_null() && event.is_valid()) {
        InputEvent* e = event.ptr();
        if (e->is_action_pressed("pause")) {
            toggle_pause();
        }
    }
}
