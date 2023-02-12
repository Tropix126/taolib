import { mergeProps, splitProps } from "solid-js";
import { Dynamic } from "solid-js/web";

import "~/styles/Button.css";

const defaultProps = {
	variant: "default",
	disabled: false
};

interface Props {
	variant?: "default" | "accent" | "danger";
	disabled?: boolean;
	[key: string]: any;
}

export default function Button(props: Props) {
	const [local, rest] = splitProps(mergeProps(defaultProps, props), [
		"children",
		"href",
		"variant",
		"disabled",
		"class"
	]);

	return (
		<Dynamic
			class={`button variant-${local.variant} ${local.class ?? ""}`}
			disabled={local.disabled}
			component={(local.href && !local.disabled) ? "a" : "button"}
			href={!local.disabled ? local.href : undefined}
			type="button"
			{...rest}
		>
			{local.children}
		</Dynamic>
	);
}