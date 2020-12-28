# Local Runner

- The `actions/checkout@v2` action requires git 1.18+

```
sudo add-apt-repository ppa:git-core/ppa
sudo apt-get update
sudo apt-get install git
```

# Integration Tests

```bash
# Local development
TAG=`git rev-parse --short HEAD` modules/core/tests/logger.sh
```
