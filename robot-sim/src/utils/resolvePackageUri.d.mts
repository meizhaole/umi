export type PackageMappings = Readonly<Record<string, string>>;

export function resolvePackageUri(uri: string, packageMappings: PackageMappings): string;

export function resolvePackageUris(urdfXml: string, packageMappings: PackageMappings): string;
