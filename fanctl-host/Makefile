# fanctl host daemon
#   make                 build ./fanctl
#   sudo make install    install binary, config (kept if present), udev rule, service; (re)start
#   sudo make uninstall  stop and remove everything except /etc/fanctl.conf

CFLAGS  ?= -O2 -Wall
LDLIBS  := -ldl -lm

# fanctl.service expects the binary in /usr/local/bin
BINDIR  := /usr/local/bin
CONF    := /etc/fanctl.conf
UDEV    := /etc/udev/rules.d/99-fanctl.rules
UNIT    := /etc/systemd/system/fanctl.service

fanctl: fanctl.c
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

install: fanctl
	install -D -m 755 fanctl $(BINDIR)/fanctl
	test -e $(CONF) || install -m 644 fanctl.conf $(CONF)
	install -m 644 99-fanctl.rules $(UDEV)
	udevadm control --reload && udevadm trigger
	install -m 644 fanctl.service $(UNIT)
	systemctl daemon-reload
	systemctl enable fanctl
	systemctl restart fanctl

uninstall:
	-systemctl disable --now fanctl
	rm -f $(BINDIR)/fanctl $(UDEV) $(UNIT)
	systemctl daemon-reload
	udevadm control --reload

clean:
	rm -f fanctl

.PHONY: install uninstall clean
