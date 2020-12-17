# Developer Snippets

- The [act](https://github.com/nektos/act) tool is useful for iterating on Github actions locally.
- When working with `act`, some actions rely on a Github access token in the GITHUB_TOKEN secret (even though our repositories are public). I recommend issuing a [Personal Access Token](https://docs.github.com/en/free-pro-team@latest/github/authenticating-to-github/creating-a-personal-access-token) **with no scopes**, and storing it at `~/.config/github/token`.

```
# Run act, with full Github runner environment
# Warning: nektos/act-environments-ubuntu:18.04 is an 18GB+ image
act --platform ubuntu-latest=nektos/act-environments-ubuntu:18.04 --secret GITHUB_TOKEN=`cat ~/.config/github/token` --secret DOCKERHUB_USERNAME=<username> --secret DOCKERHUB_TOKEN=`cat ~/.config/dockerhub/token`
```

```
# Build a docker image locally
cd docker
docker build -f base.Dockerfile -t farmng/base:`git rev-parse --short HEAD` ..

# Push to Dockerhub
docker login --username=<username>
docker push farmng/base:`git rev-parse --short HEAD`
```

# Ideas

- Release artifacts
  - https://github.com/docker/build-push-action/issues/147
