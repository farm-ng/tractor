import { TarReader } from "./TarReader";

function extension(filename: string): string {
  return filename.substr(filename.lastIndexOf(".") + 1);
}

// TODO: Use a proper mime types library
function mimeType(filename: string): string | undefined {
  const mimeTypes: Record<string, string> = {
    jpg: "image/jpeg",
    jpeg: "image/jpeg",
    json: "application/json",
    png: "image/png"
  };
  return mimeTypes[extension(filename)];
}

export interface ResourceArchive {
  getFileInfo(): Promise<string[]>;
  getDataUrl(path: string): Promise<string>;
  getBlob(path: string): Promise<Blob>;
  getJson<T>(path: string): Promise<T>;
}

export class HttpResourceArchive {
  constructor(private endpoint: string) {}

  public async getFileInfo(): Promise<string[]> {
    return [];
  }

  public async getBlob(path: string): Promise<Blob> {
    const response = await fetch(`${this.endpoint}/${path}`, {
      method: "GET"
    });
    return await response.blob();
  }

  public async getJson<T>(path: string): Promise<T> {
    const response = await fetch(`${this.endpoint}/${path}`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json"
      }
    });
    return await response.json();
  }

  public async getDataUrl(path: string): Promise<string> {
    return `${this.endpoint}/${path}`;
  }
}

export class TarResourceArchive {
  tarReader: TarReader;
  constructor(file: File) {
    this.tarReader = new TarReader(file);
  }

  public async getFileInfo(): Promise<string[]> {
    return await this.tarReader.getFileInfo();
  }

  public async getBlob(path: string): Promise<Blob> {
    return await this.tarReader.getFileBlob(path);
  }

  public async getJson<T>(path: string): Promise<T> {
    const reader = new FileReader();
    const blob = await this.tarReader.getFileBlob(path, {
      type: mimeType(path)
    });
    return new Promise((resolve, reject) => {
      reader.onload = (e: ProgressEvent<FileReader>) => {
        const result = e.target?.result;
        if (typeof result === "string") {
          resolve(JSON.parse(result) as T);
        } else {
          reject(`failed to readAsDataURL ${path}`);
        }
      };
      reader.readAsText(blob);
    });
  }

  public async getDataUrl(path: string): Promise<string> {
    const reader = new FileReader();
    const blob = await this.tarReader.getFileBlob(path, {
      type: mimeType(path)
    });
    return new Promise((resolve, reject) => {
      reader.onload = (e: ProgressEvent<FileReader>) => {
        const result = e.target?.result;
        if (typeof result === "string") {
          resolve(result as string);
        } else {
          reject(`failed to readAsDataURL ${path}`);
        }
      };
      reader.readAsDataURL(blob);
    });
  }
}
