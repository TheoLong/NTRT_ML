link_libraries(tgcreator 
	       core
              learningSpines)

add_library(SuperballModelContact
	    SuperballModelContact.cpp)

add_executable(AppSuperballContact
    SuperballModelContact.cpp
    AppSuperballContact.cpp) 
