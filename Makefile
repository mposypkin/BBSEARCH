all dep clean indent tests::
	cd spacefill && $(MAKE) $@ && cd .. 

doc: indent doxy

doxy:
	mkdir -p doc/html &&\
	doxygen doxy.conf

clean::
	rm -rf *~ PI* core bin/* obj/* tmp *.log