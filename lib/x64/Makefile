MKDIR=mkdir
CCC=g++

# Object Directory
OBJECTDIR=./obj
TARGET= ./test

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/main.o \

# C Compiler Flags
CFLAGS= -fPIC -MMD -MP -MF $@.d -fvisibility=hidden -Wall 
LDLIBSOPTIONS=-lpthread -fPIC -lpxproc -L./ -ldl
INC=


all: $(TARGET)

clean:
	rm -f $(OBJECTDIR)/*.o  $(OBJECTDIR)/*.o.d $(TARGET)

$(OBJECTDIR): 
	@mkdir -p "$@";

$(TARGET): $(OBJECTDIR) $(OBJECTFILES)
	@$(CCC) -o $(TARGET) -fPIC $(OBJECTFILES) $(LDLIBSOPTIONS) 

$(OBJECTDIR)/%.o: %.cpp
	@$(CCC) $(INC) $(CFLAGS) -o $@ -c $<


