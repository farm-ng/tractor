ifdef TEST_FILTER
	PY_TEST_FILTER=-m $(TEST_FILTER)
	JS_TEST_FILTER=:$(TEST_FILTER)
endif

frontend:
	cd app/frontend && yarn

protos:
	scripts/build-protos.sh

test:
	./env.sh pytest $(PY_TEST_FILTER)
	cd app/frontend && yarn test$(JS_TEST_FILTER)

test-integration:
	scripts/test-integration.sh

.PHONY: protos test test-integration
