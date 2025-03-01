set(EXTRA_COMPONENT_DIRS components)

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS  
    "../components/main_library" 
    
)   

