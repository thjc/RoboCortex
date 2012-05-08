all:
	./rc-make.sh

install:
	install -D bin/srv $(DESTDIR)/usr/bin/srv
	install -D bin/cli $(DESTDIR)/usr/bin/cli
	false
