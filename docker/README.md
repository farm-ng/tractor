# Developer Snippets

- The [act](https://github.com/nektos/act) tool is useful for iterating on Github actions locally.
- When working with `act`, some actions rely on a Github access token in the GITHUB_TOKEN secret (even though our repositories are public). I recommend issuing a [Personal Access Token](https://docs.github.com/en/free-pro-team@latest/github/authenticating-to-github/creating-a-personal-access-token) **with no scopes**, and storing it at `~/.config/github/token`.

```
# Run act, with full Github runner environment
# Warning: nektos/act-environments-ubuntu:18.04 is an 18GB+ image
act --platform ubuntu-latest=nektos/act-environments-ubuntu:18.04 --secret GITHUB_TOKEN=`cat ~/.config/github/token` --secret DOCKERHUB_USERNAME=<username> --secret DOCKERHUB_TOKEN=`cat ~/.config/dockerhub/token`
```


```
# Build a devel docker image locally
cd docker/devel
./build_devel.sh

# Push to Dockerhub
docker login --username=<username>
docker push farmng/devel:`git rev-parse --short HEAD`
```

When you push a new ``farmng/devel``, make sure to reference it in the ``base.Dockerfile`` ``FROM`` line.


```
# Build a docker image locally
cd docker
./build_base.sh

# Push to Dockerhub
docker login --username=<username>
docker push farmng/base:`git rev-parse --short HEAD`
```

# Ideas

- Release artifacts
  - https://github.com/docker/build-push-action/issues/147
