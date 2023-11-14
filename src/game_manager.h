#ifndef GAME_MANAGER_H
#define GAME_MANAGER_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/input.hpp>
#include <godot_cpp/classes/input_event.hpp>

namespace godot {
    class GameManager : public Node {
        GDCLASS(GameManager, Node)
    public:
        enum State {
            PAUSED,
            PLAYING,
            MENU,
        };
    private:
        static GameManager* instance;
        State game_state_;

        bool is_valid_state(State s);
        void set_game_state(State s);
    protected:
        static void _bind_methods();
        
    public:
        GameManager();
        ~GameManager();

        static GameManager* get_singleton();

        void toggle_pause();

        void _ready();
        void _process(double delta);
        void _input(const Ref<InputEvent>& event);
    };
}

#endif