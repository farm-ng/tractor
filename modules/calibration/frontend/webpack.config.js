/* eslint-disable no-undef */
/* eslint-disable @typescript-eslint/explicit-function-return-type */
/* eslint-disable @typescript-eslint/no-var-requires */
/* global require __dirname */
const path = require("path");
const webpack = require("webpack");
const postcssNormalize = require("postcss-normalize");
const getCSSModuleLocalIdent = require("react-dev-utils/getCSSModuleLocalIdent");
const MiniCssExtractPlugin = require("mini-css-extract-plugin");
const HtmlWebpackPlugin = require("html-webpack-plugin");
const { CleanWebpackPlugin } = require("clean-webpack-plugin");

// Source maps are resource heavy and can cause out of memory issue for large source files.
const shouldUseSourceMap = process.env.GENERATE_SOURCEMAP !== "false";

// CSS/SASS regexes
const cssRegex = /\.css$/;
const cssModuleRegex = /\.module\.css$/;
const sassRegex = /\.(scss|sass)$/;
const sassModuleRegex = /\.module\.(scss|sass)$/;

// Any images less than this limit (in bytes) will be inlined
const imageInlineSizeLimit = parseInt(
  process.env.IMAGE_INLINE_SIZE_LIMIT || "10000"
);

module.exports = function (env, argv) {
  const isEnvDevelopment = argv.mode !== "production";
  const isEnvProduction = argv.mode === "production";

  // Common function to get style loaders
  const getStyleLoaders = (cssOptions, preProcessor) => {
    const loaders = [
      isEnvDevelopment && require.resolve("style-loader"),
      isEnvProduction && { loader: MiniCssExtractPlugin.loader },
      {
        loader: require.resolve("css-loader"),
        options: cssOptions,
      },
      {
        // Options for PostCSS as we reference these options twice
        // Adds vendor prefixing based on your specified browser support in
        // package.json
        loader: require.resolve("postcss-loader"),
        options: {
          postcssOptions: {
            // Necessary for external CSS imports to work
            // https://github.com/facebook/create-react-app/issues/2677
            ident: "postcss",
            plugins: () => [
              require("postcss-flexbugs-fixes"),
              require("postcss-preset-env")({
                autoprefixer: {
                  flexbox: "no-2009",
                },
                stage: 3,
              }),
              // Adds PostCSS Normalize as the reset css with default options,
              // so that it honors browserslist config in package.json
              // which in turn let's users customize the target behavior as per their needs.
              postcssNormalize(),
            ],
            sourceMap: isEnvProduction ? shouldUseSourceMap : isEnvDevelopment,
          },
        },
      },
    ].filter(Boolean);
    if (preProcessor) {
      loaders.push(
        {
          loader: require.resolve("resolve-url-loader"),
          options: {
            sourceMap: isEnvProduction ? shouldUseSourceMap : isEnvDevelopment,
            root: path.resolve(__dirname, "src/"),
          },
        },
        {
          loader: require.resolve(preProcessor),
          options: {
            sourceMap: true,
          },
        }
      );
    }
    return loaders;
  };

  return {
    mode: isEnvProduction ? "production" : isEnvDevelopment && "development",
    // Fail immediately on error in production
    bail: isEnvProduction,
    output: {
      filename: "bundle.js",
      path: path.resolve(__dirname, "dist/"),
      // Add /* filename */ comments to generated require()s in the output.
      pathinfo: isEnvDevelopment,
      // Required so that /bundle.js is served even at non-root paths
      publicPath: "/",
      // Use the future version of asset emitting logic,
      // which allows freeing memory of assets after emitting
      futureEmitAssets: true,
    },
    optimization: {
      // none yet
    },
    devServer: {
      historyApiFallback: true,
    },
    devtool: isEnvProduction
      ? shouldUseSourceMap
        ? "source-map"
        : false
      : isEnvDevelopment && "cheap-module-source-map",
    entry: isEnvDevelopment
      ? [
          require.resolve("webpack-dev-server/client") + "?/",
          require.resolve("webpack/hot/dev-server"),
          path.resolve(__dirname, "src/index.tsx"),
        ]
      : path.resolve(__dirname, "src/index.tsx"),
    resolve: {
      extensions: ["*", ".js", ".jsx", ".ts", ".tsx"],

      // Force all React references to resolve to a single module,
      // to avoid including an instance of React that conflicts with dependency
      // https://github.com/facebook/react/issues/14257#issuecomment-508808246
      alias: { react: require.resolve("react") },
    },
    module: {
      // makes missing exports an error instead of warning
      strictExportPresence: true,
      rules: [
        {
          // "oneOf" will traverse all following loaders until one will
          // match the requirements. When no loader matches it will fall
          // back to the "file" loader at the end of the loader list.
          oneOf: [
            // "url-loader" works like "file-loader" except that it embeds assets
            // smaller than specified limit in bytes as data URLs to avoid requests.
            {
              test: [/\.bmp$/, /\.gif$/, /\.jpe?g$/, /\.png$/],
              loader: require.resolve("url-loader"),
              options: {
                limit: imageInlineSizeLimit,
                name: "static/media/[name].[hash:8].[ext]",
              },
            },
            // Compile Typescript
            // Could potentially be replaced by babel7, which supports TS compilation
            {
              test: /\.(js|jsx|ts|tsx)$/,
              exclude: /(node_modules|bower_components)/,
              loader: "ts-loader",
            },
            // "postcss" loader applies autoprefixer to our CSS.
            // "css" loader resolves paths in CSS and adds assets as dependencies.
            // "style" loader turns CSS into JS modules that inject <style> tags.
            // In production, we use MiniCSSExtractPlugin to extract that CSS
            // to a file, but in development "style" loader enables hot editing
            // of CSS.
            // By default we support CSS Modules with the extension .module.css
            {
              test: cssRegex,
              exclude: cssModuleRegex,
              use: getStyleLoaders({
                importLoaders: 1,
                sourceMap: isEnvProduction
                  ? shouldUseSourceMap
                  : isEnvDevelopment,
              }),
              // Don't consider CSS imports dead code even if the
              // containing package claims to have no side effects.
              // Remove this when webpack adds a warning or an error for this.
              // See https://github.com/webpack/webpack/issues/6571
              sideEffects: true,
            },
            // Adds support for CSS Modules (https://github.com/css-modules/css-modules)
            // using the extension .module.css
            {
              test: cssModuleRegex,
              use: getStyleLoaders({
                importLoaders: 1,
                sourceMap: isEnvProduction
                  ? shouldUseSourceMap
                  : isEnvDevelopment,
                modules: {
                  getLocalIdent: getCSSModuleLocalIdent,
                },
              }),
            },
            // Opt-in support for SASS (using .scss or .sass extensions).
            // By default we support SASS Modules with the
            // extensions .module.scss or .module.sass
            {
              test: sassRegex,
              exclude: sassModuleRegex,
              use: getStyleLoaders(
                {
                  importLoaders: 3,
                  sourceMap: isEnvProduction
                    ? shouldUseSourceMap
                    : isEnvDevelopment,
                },
                "sass-loader"
              ),
              // Don't consider CSS imports dead code even if the
              // containing package claims to have no side effects.
              // Remove this when webpack adds a warning or an error for this.
              // See https://github.com/webpack/webpack/issues/6571
              sideEffects: true,
            },
            // Adds support for CSS Modules, but using SASS
            // using the extension .module.scss or .module.sass
            {
              test: sassModuleRegex,
              use: getStyleLoaders(
                {
                  importLoaders: 3,
                  sourceMap: isEnvProduction
                    ? shouldUseSourceMap
                    : isEnvDevelopment,
                  modules: {
                    getLocalIdent: getCSSModuleLocalIdent,
                  },
                },
                "sass-loader"
              ),
            },
            // A catch-all loader for other types of files not captured above.
            // In development, serves assets with WebpackDevServer.
            // In production, copies assets to the `build` folder.
            {
              loader: require.resolve("file-loader"),
              // Exclude `js` files to keep "css" loader working as it injects
              // its runtime that would otherwise be processed through "file" loader.
              // Also exclude `html` and `json` extensions so they get processed
              // by webpacks internal loaders.
              exclude: [/\.(js|mjs|jsx|ts|tsx)$/, /\.html$/, /\.json$/],
              options: {
                name: "static/media/[name].[hash:8].[ext]",
              },
            },
            // ** STOP ** Are you adding a new loader?
            // Make sure to add the new loader(s) before the "file" loader.
          ],
        },
      ],
    },
    plugins: [
      // Clears the output directory before each build
      new CleanWebpackPlugin(),
      // Allows us to forward build-time environment variables
      new webpack.EnvironmentPlugin(["BASE_URL"]),
      // Generates an `index.html` file with the <script> injected.
      new HtmlWebpackPlugin({
        inject: true,
        template: "src/index.html",
      }),
      // This is necessary to emit hot updates (CSS and Fast Refresh):
      isEnvDevelopment && new webpack.HotModuleReplacementPlugin(),
      // Generate separate .css file(s) in production
      isEnvProduction &&
        new MiniCssExtractPlugin({
          filename: "static/css/[name].[contenthash:8].css",
          chunkFilename: "static/css/[name].[contenthash:8].chunk.css",
        }),
    ].filter(Boolean),
  };
};
