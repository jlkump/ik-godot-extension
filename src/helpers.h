#ifndef HELPERS_H
#define HELPERS_H


// Use this in _bind_methods() to bind a getter, setter, and property in editor
// name is the name of the variable (which is also assumed to have getters and setters with get_name and set_name)
// property_info must be of type PropertyInfo
#define BIND_GETTER_SETTER(class, name, property_info) \
    ClassDB::bind_method(D_METHOD("get_" #name), &##class::get_##name); \
    ClassDB::bind_method(D_METHOD("set_" #name, "p_" #name), &##class::set_##name); \
    ClassDB::add_property(#class, property_info, "set_" #name, "get_" #name);

// Use this to quickly define a getter and setter that simply returns a private
// member variable. More complex getters/setters will have to be done manually
#define DEFINE_GETTER_SETTER(class, type, name) \
    type class::get_##name() const { return name; } \
    void class::set_##name(const type p_##name) { name = p_##name; }

// Use this in the header file to quickly declare getters and setters for
// a member variable of given type and name
#define DECLARE_GETTER_SETTER(type, name) \
    type get_##name() const; \
    void set_##name(const type p_##name);

#endif