
# Directories
SRCDIR=.
LIBDIR=.

# Files
CSOURCES=$(shell find $(SRCDIR) -name "*.c")
CXXSOURCES=$(shell find $(SRCDIR) -name "*.cpp")
COBJECTS=$(patsubst %.c,%.o,$(CSOURCES))
CXXOBJECTS=$(patsubst %.cpp,%.o,$(CXXSOURCES))
LIBNAME=$(notdir $(shell pwd))

# Targets
lib: $(LIBNAME).a

$(LIBNAME).a: $(COBJECTS) $(CXXOBJECTS)
	ar rcs $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $< $(IFLAGS)

%.o: %.cpp
	$(CXX) $(CFLAGS) -c -o $@ $< $(IFLAGS)

clean:
	rm -f $(LIBNAME).a $(COBJECTS) $(CXXOBJECTS)