// TODO: Support common tarball extensions: UStar, PaxHeaders, etc.

const filenameMaxLength = 1024;
const fileSizeOffset = 124;
const fileTypeOffset = 156;

interface FileInfo {
  name: string;
  type: string;
  size: number;
  headerOffset: number;
}

function readString(buffer: ArrayBuffer, offset: number, size: number): string {
  const view = new Uint8Array(buffer, offset, size);
  let i = 0;
  let result = "";
  while (view[i] != 0) {
    result += String.fromCharCode(view[i]);
    i++;
  }
  return result;
}

function readFileName(buffer: ArrayBuffer, headerOffset: number): string {
  return readString(buffer, headerOffset, filenameMaxLength);
}

function readFileType(buffer: ArrayBuffer, headerOffset: number): string {
  const view = new Uint8Array(buffer, headerOffset + fileTypeOffset, 1);
  const typeChar = String.fromCharCode(view[0]);
  if (typeChar == "0") {
    return "file";
  } else if (typeChar == "5") {
    return "directory";
  } else {
    return typeChar;
  }
}

function readFileSize(buffer: ArrayBuffer, headerOffset: number): number {
  const view = new Uint8Array(buffer, headerOffset + fileSizeOffset, 12);
  let result = "";
  for (let i = 0; i < 11; i++) {
    result += String.fromCharCode(view[i]);
  }
  return parseInt(result, 8);
}

function readFileInfo(buffer: ArrayBuffer): FileInfo[] {
  let offset = 0;
  let fileSize = 0;
  let fileName = "";
  let fileType = null;
  const fileInfo: FileInfo[] = [];
  while (offset < buffer.byteLength - 512) {
    fileName = readFileName(buffer, offset);
    if (fileName.length == 0) {
      break;
    }
    fileType = readFileType(buffer, offset);
    fileSize = readFileSize(buffer, offset);

    fileInfo.push({
      name: fileName,
      type: fileType,
      size: fileSize,
      headerOffset: offset
    });

    offset += 512 + 512 * Math.trunc(fileSize / 512);
    if (fileSize % 512) {
      offset += 512;
    }
  }
  return fileInfo;
}

function readFileBlob(
  buffer: ArrayBuffer,
  offset: number,
  size: number,
  blobProperties?: BlobPropertyBag
): Blob {
  const view = new Uint8Array(buffer, offset, size);
  const blob = new Blob([view], blobProperties);
  return blob;
}

export function normalizeTarPath(path: string): string {
  return path.substring(path.indexOf("/") + 1);
}

export class TarReader {
  private data: Promise<{ fileInfo: FileInfo[]; buffer: ArrayBuffer }>;

  constructor(file: File) {
    const reader = new FileReader();
    this.data = new Promise((resolve, reject) => {
      reader.onload = (event) => {
        const buffer = event.target?.result;
        if (!(buffer instanceof ArrayBuffer)) {
          reject(`failed to readAsArrayBuffer ${file.name}`);
        } else {
          const fileInfo = readFileInfo(buffer);
          resolve({ buffer, fileInfo });
        }
      };
      reader.readAsArrayBuffer(file);
    });
  }

  public async getFileBlob(
    fileName: string,
    blobProperties?: BlobPropertyBag
  ): Promise<Blob> {
    const { buffer, fileInfo } = await this.data;
    const info = fileInfo.find((_) => fileName === normalizeTarPath(_.name));
    if (!info) {
      return Promise.reject(`${fileName} not found in tar directory`);
    }
    return readFileBlob(
      buffer,
      info.headerOffset + 512,
      info.size,
      blobProperties
    );
  }

  public async getFileInfo(): Promise<string[]> {
    const { fileInfo } = await this.data;
    return fileInfo.map((_) => _.name);
  }
}
