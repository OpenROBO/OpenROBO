CC       = g++
INCLUDE += -I./header
CFLAGS  += -Wall -g -MMD -MP
LDFLAGS += -lpthread
SRC_DIR  = ./source
OBJ_DIR  = ./build
SOURCES  = $(shell ls $(SRC_DIR)/*.cpp)
OBJS     = $(subst $(SRC_DIR),$(OBJ_DIR), $(SOURCES:.cpp=.o))
DEPENDS  = $(OBJS:.o=.d)

RMDIR = rm -rf

#CFLAGS += -DOPENROBO_TRACE_MESSAGE

.PHONY: all
all: $(TARGET)

$(TARGET): $(OBJS) $(LIBS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS) $(LIBS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@if [ ! -d $(OBJ_DIR) ]; \
		then echo "mkdir -p $(OBJ_DIR)"; mkdir -p $(OBJ_DIR); \
	fi
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

.PHONY: clean
clean:
	$(RM) $(TARGET)
	$(RMDIR) $(OBJ_DIR)

.PHONY: run
run: all
	@while : ; do \
	./$(TARGET) ; \
	res=$${?} ; \
	echo "return code is $${res}" ; \
	if [ $${res} -ne 0 ]; then \
	exit $${res} ; \
	fi ; \
	echo ; \
	count=5 ; \
	while [ $${count} != 0 ] ; do \
	echo -n "$${count} ... " ; \
	sleep 1 ; \
	count=`expr $${count} - 1` ; \
	done ; \
	echo ; \
	echo ; \
	done

.PHONY: runi
runi: all
	@while : ; do \
	./$(TARGET) ; \
	res=$${?} ; \
	echo "return code is $${res}" ; \
	echo ; \
	count=5 ; \
	while [ $${count} != 0 ] ; do \
	echo -n "$${count} ... " ; \
	sleep 1 ; \
	count=`expr $${count} - 1` ; \
	done ; \
	echo ; \
	echo ; \
	done

.PHONY: help h
help:
	@echo "make run; execute once after make"
	@echo "make runi; execute indefinitely(infinity) after make"
	@echo "make h; same as \"make help\""

h: help

-include $(DEPENDS)
