declare module "*.scss" {
  const styles: { [className: string]: string };
  export default styles;
}

declare module "*.css" {
  const styles: { [className: string]: string };
  export default styles;
}

declare module "*.stl" {
  const content: string;
  export default content;
}
